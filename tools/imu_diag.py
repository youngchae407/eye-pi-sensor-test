#!/usr/bin/env python3
"""BNO086 I2C 진단 도구 (라이브러리 없이 /dev/i2c-1 을 직접 읽고 씀).

센서가 리셋 후 어떤 패킷을 보내는지, 제품 ID 요청과 가속도계 켜기에 어떻게 응답하는지
패킷 단위로 찍어 줍니다. 서버(server.py)를 먼저 멈추고 실행하세요.

    python tools/imu_diag.py                 # 기본 주소 0x4B
    python tools/imu_diag.py --reset-pin 24  # RST 를 GPIO24 에 연결했을 때 하드웨어 리셋부터
    python tools/imu_diag.py --library       # 라이브러리 debug=True 초기화도 이어서 실행

읽는 법: '채널 0' = SHTP 명령(광고/오류 목록), '채널 1' = 실행(리셋 완료 0x01),
'채널 2' = 제어(제품 ID 0xF8, 기능 응답 0xFC), '채널 3' = 센서 데이터(0xFB 타임스탬프 + 리포트).
"""
import argparse
import fcntl
import os
import struct
import sys
import time

I2C_SLAVE = 0x0703
CHANNEL_NAMES = {0: "SHTP명령", 1: "실행", 2: "제어", 3: "센서", 4: "웨이크센서", 5: "자이로RV"}


class RawBno:
    def __init__(self, bus, addr):
        self.fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, addr)
        self.seq = [0] * 6
        self.counts = {}
        self.io_errors = 0

    def send(self, channel, payload):
        data = struct.pack("<HBB", len(payload) + 4, channel, self.seq[channel]) + bytes(payload)
        self.seq[channel] = (self.seq[channel] + 1) % 256
        os.write(self.fd, data)
        print(f"  → 보냄  채널 {channel}({CHANNEL_NAMES.get(channel, '?')}) [{data.hex(' ')}]")

    def read_packet(self):
        """패킷 하나를 읽어 (채널, 데이터) 를 돌려준다. 읽을 게 없으면 None."""
        try:
            header = os.read(self.fd, 4)
        except OSError as e:
            self.io_errors += 1
            print(f"  ! I2C 읽기 오류: {e}")
            return None
        length = struct.unpack_from("<H", header)[0]
        if length == 0xFFFF:
            print(f"  ! 헤더가 FF FF: 센서가 응답하지 않거나 선이 불안정함 [{header.hex(' ')}]")
            return None
        length &= 0x7FFF
        if length == 0:
            return None
        if length > 4:
            try:
                full = os.read(self.fd, length)  # 센서는 헤더부터 다시 보내 준다
            except OSError as e:
                self.io_errors += 1
                print(f"  ! I2C 읽기 오류(본문): {e}")
                return None
        else:
            full = header
        channel, seq = full[2], full[3]
        body = full[4:]
        self.counts[channel] = self.counts.get(channel, 0) + 1
        shown = body[:32].hex(" ") + (" …" if len(body) > 32 else "")
        print(f"  ← 받음  채널 {channel}({CHANNEL_NAMES.get(channel, '?')}) seq={seq} "
              f"{len(body)}바이트 [{shown}]")
        return channel, body

    def drain(self, seconds):
        packets = []
        end = time.time() + seconds
        while time.time() < end:
            p = self.read_packet()
            if p is None:
                time.sleep(0.02)
            else:
                packets.append(p)
        return packets


def hardware_reset(bcm):
    import board
    import digitalio
    pin = digitalio.DigitalInOut(getattr(board, f"D{bcm}"))
    pin.direction = digitalio.Direction.OUTPUT
    pin.value = True
    time.sleep(0.01)
    pin.value = False
    time.sleep(0.01)
    pin.value = True
    pin.deinit()
    print(f"[하드웨어 리셋] GPIO{bcm} 에 LOW 펄스를 보냈습니다")


def raw_test(args):
    print(f"=== 1) 원시 I2C 통신 (주소 {hex(args.address)}) ===")
    if args.reset_pin is not None:
        hardware_reset(args.reset_pin)
        time.sleep(0.3)
    dev = RawBno(args.bus, args.address)

    print("\n[a] 센서가 이미 보내려고 쌓아 둔 패킷 (리셋 직후라면 광고 패킷이 와야 정상)")
    dev.drain(1.0)

    print("\n[b] 소프트 리셋 (채널 1 에 0x01) 후 1.5초 동안 수신")
    dev.send(1, [0x01])
    time.sleep(0.5)
    after_reset = dev.drain(1.5)
    if not any(ch == 0 for ch, _ in after_reset):
        print("  ? 리셋 뒤 광고 패킷(채널 0)이 오지 않았습니다")

    print("\n[c] 제품 ID 요청 (채널 2 에 F9 00)")
    dev.send(2, [0xF9, 0x00])
    got_id = False
    for ch, body in dev.drain(1.0):
        if ch == 2 and body[:1] == b"\xf8" and len(body) >= 16:
            got_id = True
            sw_major, sw_minor = body[2], body[3]
            part = struct.unpack_from("<I", body, 4)[0]
            build = struct.unpack_from("<I", body, 8)[0]
            patch = struct.unpack_from("<H", body, 12)[0]
            print(f"  ✓ 제품 ID: 부품번호 {part}, 펌웨어 {sw_major}.{sw_minor}.{patch} (빌드 {build})")
    if not got_id:
        print("  ✗ 제품 ID 응답을 받지 못했습니다")

    print("\n[d] 가속도계 켜기 (채널 2, Set Feature 0xFD, 100ms 간격) 후 1.5초 수신")
    feature = bytearray(17)
    feature[0], feature[1] = 0xFD, 0x01
    struct.pack_into("<I", feature, 5, 100_000)
    dev.send(2, feature)
    reports = dev.drain(1.5)
    sensor_packets = [b for ch, b in reports if ch == 3]
    errors = [b for ch, b in reports if ch == 0 and b[:1] == b"\x01"]

    print("\n--- 요약 ---")
    print("채널별 받은 패킷 수:", {f"{c}({CHANNEL_NAMES.get(c, '?')})": n for c, n in sorted(dev.counts.items())})
    print("I2C 읽기 오류:", dev.io_errors)
    print("제품 ID:", "OK" if got_id else "실패")
    print("가속도 데이터 패킷:", len(sensor_packets))
    if errors:
        print("채널 0 의 01 xx 메시지(SHTP 오류 보고로 추정):", [e.hex(" ") for e in errors])
        print("  → 라이브러리가 이걸 가속도 리포트로 오해해 'Unprocessable Batch bytes' 를 냅니다.")
        print("    imu_init.patch_library() 가 이 패킷을 건너뜁니다.")
    os.close(dev.fd)
    return got_id and bool(sensor_packets)


def library_test(args):
    print("\n=== 2) 라이브러리 debug=True 초기화 (패치 없이 원본 그대로) ===")
    import board
    import busio
    from adafruit_bno08x import BNO_REPORT_ACCELEROMETER
    from adafruit_bno08x.i2c import BNO08X_I2C
    i2c = busio.I2C(board.SCL, board.SDA)
    reset = None
    if args.reset_pin is not None:
        import digitalio
        reset = digitalio.DigitalInOut(getattr(board, f"D{args.reset_pin}"))
    try:
        bno = BNO08X_I2C(i2c, reset=reset, address=args.address, debug=True)
        bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        print("[OK] 원본 라이브러리로도 가속도:", bno.acceleration)
        return True
    except Exception as e:  # noqa: BLE001
        print(f"[FAIL] {type(e).__name__}: {e}")
        return False


def main():
    ap = argparse.ArgumentParser(description="BNO086 I2C 진단")
    ap.add_argument("--bus", type=int, default=1)
    ap.add_argument("--address", type=lambda x: int(x, 0), default=0x4B)
    ap.add_argument("--reset-pin", type=int, default=None, metavar="BCM")
    ap.add_argument("--library", action="store_true", help="라이브러리 debug 초기화도 실행")
    args = ap.parse_args()

    ok = raw_test(args)
    print("\n원시 통신 결과:", "정상 ✅" if ok else "문제 있음 ❌")
    if args.library:
        library_test(args)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
