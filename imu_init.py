"""BNO086 초기화 도우미 (server.py 와 test_imu.py 가 같이 씁니다).

BNO08x는 Pi의 I2C에서 초기화 도중 가끔 `RuntimeError: ('Unprocessable Batch bytes', N)` 를
내며 실패합니다. 센서가 보낸 짧은 상태 패킷을 라이브러리가 센서 데이터로 잘못 읽는 것으로,
센서를 다시 리셋하고 처음부터 재시도하면 대부분 지나갑니다.

원인(라이브러리 v1.3.3 소스로 확인): enable_feature() 가 패킷을 처리할 때 채널을 가리지 않아,
센서가 SHTP 명령 채널(0)로 보낸 2바이트 메시지(예: 01 0E)의 첫 바이트 0x01 을
가속도계 리포트 ID 로 착각하고 10바이트를 기대하다 예외를 냅니다.
patch_library() 가 채널 0/1 패킷은 센서 데이터로 해석하지 않고 건너뛰게 고칩니다.

이 모듈은 '객체 생성 + 기능 켜기'를 한 묶음으로 묶어 통째로 재시도하고,
라이브러리가 실패할 때마다 찍는 패킷 덤프도 조용하게 만듭니다.
"""
import importlib
import time

# patch_library() 가 건너뛴 패킷 기록: (채널, 데이터 hex) — 진단용
skipped_packets = []
_SKIP_CHANNELS = (0, 1)  # SHTP 명령 채널, 실행(EXE) 채널: 센서 리포트가 아님


def patch_library():
    """BNO08X._handle_packet 이 채널 0/1 패킷을 센서 데이터로 해석하지 않게 한다 (여러 번 불러도 안전)."""
    lib = importlib.import_module("adafruit_bno08x")
    cls = lib.BNO08X
    if getattr(cls, "_eye_pi_patched", False):
        return lib
    original = cls._handle_packet

    def _handle_packet(self, packet):
        if packet.channel_number in _SKIP_CHANNELS:
            skipped_packets.append((packet.channel_number, bytes(packet.data).hex(" ")))
            del skipped_packets[:-50]  # 최근 50개만 보관
            return
        original(self, packet)

    cls._handle_packet = _handle_packet
    cls._eye_pi_patched = True
    return lib


def quiet_library():
    """라이브러리를 패치하고, 실패 시 print(packet) 으로 쏟아내는 덤프를 끈다."""
    lib = patch_library()
    lib.print = lambda *a, **k: None  # 모듈 전역 print 를 덮어써서 내장 print 대신 쓰이게 함
    return lib


def open_reset_pin(bcm):
    """RST 핀을 Pi의 GPIO(BCM 번호)에 연결했을 때 하드웨어 리셋용 핀을 만든다."""
    if bcm is None:
        return None
    import board
    import digitalio
    pin = digitalio.DigitalInOut(getattr(board, f"D{bcm}"))
    return pin


def bring_up(i2c, candidates, features, log, reset_pin=None, tries=5):
    """센서를 만들고 features 를 켠다. (bno, addr) 를 돌려주고, 끝내 실패하면 마지막 예외를 낸다.

    log(msg, level) 로 진행 상황을 알린다.
    """
    from adafruit_bno08x.i2c import BNO08X_I2C

    last = None
    for addr in candidates:
        for attempt in range(1, tries + 1):
            try:
                bno = BNO08X_I2C(i2c, reset=reset_pin, address=addr)
                time.sleep(0.2)
                for feature in features:
                    bno.enable_feature(feature)
                    time.sleep(0.05)
                if skipped_packets:
                    log(f"IMU 명령 채널 메시지 {len(skipped_packets)}개 무시함 "
                        f"(최근: 채널 {skipped_packets[-1][0]} [{skipped_packets[-1][1]}])", "info")
                return bno, addr
            except Exception as e:  # noqa: BLE001
                last = e
                log(f"IMU 초기화 재시도 {attempt}/{tries} (주소 {hex(addr)}): {type(e).__name__}: {e}", "warn")
                time.sleep(0.5 + 0.3 * attempt)  # 점점 더 길게 쉬었다 다시 리셋
    raise last if last else RuntimeError("BNO086 초기화 실패")
