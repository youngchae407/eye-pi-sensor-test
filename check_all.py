#!/usr/bin/env python3
"""마이크 + IMU 테스트를 차례로 실행하고 요약합니다.

    python check_all.py
    python check_all.py --mic-duration 8 --imu-duration 15
"""
import argparse
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def run(script, *extra):
    print("\n" + "=" * 60)
    code = subprocess.call([sys.executable, str(HERE / script), *extra])
    return code


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mic-duration", default="5")
    ap.add_argument("--imu-duration", default="10")
    args = ap.parse_args()

    mic = run("test_mic.py", "--duration", args.mic_duration)
    imu = run("test_imu.py", "--duration", args.imu_duration)

    names = {0: "PASS", 1: "FAIL", 2: "ERROR(환경)"}
    print("\n" + "=" * 60)
    print("요약")
    print(f"  마이크 (I2S) : {names.get(mic, mic)}")
    print(f"  IMU (BNO086) : {names.get(imu, imu)}")
    return 0 if mic == 0 and imu == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
