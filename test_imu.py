#!/usr/bin/env python3
"""SparkFun BNO086 IMU 연결/값 수신 테스트 (I2C).

- I2C 버스를 스캔해 BNO086(0x4B 또는 0x4A)을 찾고
- 가속도 / 자이로 / 지자기 / 회전(쿼터니언 -> 오일러각)을 읽어 출력한 뒤
- 값이 정상 범위인지 PASS/FAIL로 판정합니다.

테스트 중 처음 몇 초는 보드를 책상 위에 가만히 두세요 (중력 ≈ 9.8 m/s² 확인용).
그 뒤에 천천히 돌려 보면 자이로/각도 값이 변하는 것을 볼 수 있습니다.

종료 코드: 0 = PASS, 1 = FAIL, 2 = 환경 문제(라이브러리 없음 등)
"""
import argparse
import math
import sys
import time


def quat_to_euler(i, j, k, w):
    """쿼터니언 -> (roll, pitch, yaw) 도(degree)."""
    sinr = 2.0 * (w * i + j * k)
    cosr = 1.0 - 2.0 * (i * i + j * j)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (w * j - k * i)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny = 2.0 * (w * k + i * j)
    cosy = 1.0 - 2.0 * (j * j + k * k)
    yaw = math.atan2(siny, cosy)
    return tuple(math.degrees(x) for x in (roll, pitch, yaw))


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def main():
    ap = argparse.ArgumentParser(description="BNO086 IMU 테스트")
    ap.add_argument("--address", type=lambda x: int(x, 0), default=None,
                    help="I2C 주소 (기본: 0x4B, 0x4A 순서로 시도)")
    ap.add_argument("--duration", type=float, default=10.0, help="측정 시간(초), 기본 10")
    ap.add_argument("--rate", type=float, default=5.0, help="출력 횟수(Hz), 기본 5")
    args = ap.parse_args()

    try:
        import board
        import busio
        from adafruit_bno08x import (
            BNO_REPORT_ACCELEROMETER,
            BNO_REPORT_GYROSCOPE,
            BNO_REPORT_MAGNETOMETER,
            BNO_REPORT_ROTATION_VECTOR,
        )
        from adafruit_bno08x.i2c import BNO08X_I2C
    except Exception as e:  # ImportError, NotImplementedError(비-Pi 환경) 등
        print(f"[ERROR] 라이브러리를 불러올 수 없습니다: {e}")
        print("        setup.sh를 실행했는지, 가상환경(source .venv/bin/activate)이 켜져 있는지 확인하세요.")
        return 2

    print("=== BNO086 IMU 테스트 ===")

    # 1) I2C 스캔
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
    except Exception as e:
        print(f"[FAIL] I2C 버스를 열 수 없습니다: {e}")
        print("       - raspi-config 또는 setup.sh로 I2C를 켜고 재부팅했는지 확인 (ls /dev/i2c-1)")
        return 1

    while not i2c.try_lock():
        time.sleep(0.01)
    try:
        found = i2c.scan()
    finally:
        i2c.unlock()
    print("I2C 스캔 결과:", [hex(a) for a in found] or "(없음)")

    candidates = [args.address] if args.address is not None else [0x4B, 0x4A]
    candidates = [a for a in candidates if a in found]
    if not candidates:
        print("[FAIL] BNO086(0x4B/0x4A)이 I2C 버스에서 보이지 않습니다.")
        print("       - 배선 확인: 3V3→핀1, GND→핀6, SDA→핀3(GPIO2), SCL→핀5(GPIO3)")
        print("       - 납땜/헤더 접촉 불량, SDA/SCL 바뀜 여부 확인")
        print("       - 터미널에서 `i2cdetect -y 1` 로도 확인 가능")
        return 1

    # 2) 초기화 (BNO08x는 첫 시도에서 가끔 실패하므로 재시도)
    bno = None
    addr = None
    for a in candidates:
        for attempt in range(1, 4):
            try:
                bno = BNO08X_I2C(i2c, address=a)
                addr = a
                break
            except Exception as e:
                print(f"  초기화 재시도 {attempt}/3 (addr {hex(a)}): {e}")
                time.sleep(0.5)
        if bno:
            break
    if bno is None:
        print("[FAIL] 센서를 찾았지만 초기화에 실패했습니다.")
        print("       - 센서 전원을 한 번 껐다 켜 보세요 (3V3 재연결)")
        print("       - SparkFun 보드의 PS0/PS1 점퍼가 I2C 모드(기본값)인지 확인")
        return 1
    print(f"[OK] BNO086 초기화 성공 (주소 {hex(addr)})")

    for feature in (BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE,
                    BNO_REPORT_MAGNETOMETER, BNO_REPORT_ROTATION_VECTOR):
        bno.enable_feature(feature)
    time.sleep(0.5)

    # 3) 읽기 루프
    print(f"\n{args.duration:.0f}초 동안 측정합니다. 처음엔 가만히, 그 뒤엔 천천히 돌려 보세요.\n")
    print(f"{'accel (m/s²)':>26} | {'gyro (rad/s)':>26} | {'mag (uT)':>26} | roll/pitch/yaw (deg)")

    interval = 1.0 / max(args.rate, 0.1)
    t_end = time.time() + args.duration
    accel_norms, quat_norms, gyro_norms = [], [], []
    samples = 0
    errors = 0

    while time.time() < t_end:
        try:
            ax, ay, az = bno.acceleration
            gx, gy, gz = bno.gyro
            mx, my, mz = bno.magnetic
            qi, qj, qk, qw = bno.quaternion
        except (KeyError, RuntimeError, OSError, TypeError) as e:
            # 라이브러리가 가끔 일시적인 패킷 오류를 냄 -> 건너뜀
            errors += 1
            time.sleep(0.05)
            continue

        samples += 1
        accel_norms.append(norm((ax, ay, az)))
        gyro_norms.append(norm((gx, gy, gz)))
        quat_norms.append(norm((qi, qj, qk, qw)))
        roll, pitch, yaw = quat_to_euler(qi, qj, qk, qw)

        print(f"{ax:8.2f} {ay:8.2f} {az:8.2f} | "
              f"{gx:8.3f} {gy:8.3f} {gz:8.3f} | "
              f"{mx:8.1f} {my:8.1f} {mz:8.1f} | "
              f"{roll:7.1f} {pitch:7.1f} {yaw:7.1f}")
        time.sleep(interval)

    # 4) 판정
    print("\n--- 결과 ---")
    results = []

    ok = samples >= max(3, int(args.duration * args.rate * 0.3))
    results.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] 데이터 수신: {samples}개 (오류 {errors}회)")

    if samples:
        mean_a = sum(accel_norms) / samples
        ok = 8.0 <= mean_a <= 11.5
        results.append(ok)
        print(f"[{'PASS' if ok else 'FAIL'}] 가속도 크기 평균 {mean_a:.2f} m/s² (정지 시 약 9.8 기대)")

        mean_q = sum(quat_norms) / samples
        ok = 0.95 <= mean_q <= 1.05
        results.append(ok)
        print(f"[{'PASS' if ok else 'FAIL'}] 쿼터니언 크기 평균 {mean_q:.3f} (1.0 기대)")

        max_g = max(gyro_norms)
        print(f"[INFO] 자이로 최대 {max_g:.3f} rad/s "
              f"({'움직임 감지됨' if max_g > 0.3 else '거의 움직이지 않음 - 보드를 돌려 보면 값이 변해야 함'})")

    passed = bool(results) and all(results)
    print("\n전체 결과:", "PASS ✅" if passed else "FAIL ❌")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
