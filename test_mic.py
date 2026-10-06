#!/usr/bin/env python3
"""I2S MEMS 마이크(Adafruit SPH0645 / ICS-43434) 수신 테스트.

arecord로 48kHz / 32bit / 모노 오디오를 받아서
- 마이크 카드가 인식되는지
- 데이터가 실제로 들어오는지 (멈춤/전부 0 아님)
- 소리 크기에 따라 레벨이 변하는지
를 확인하고 실시간 레벨 미터를 보여 줍니다.

사용 예:
    python test_mic.py                 # 5초 측정
    python test_mic.py --duration 10 --save out.wav
    python test_mic.py --require-sound   # 소리 변화가 없으면 FAIL 처리

종료 코드: 0 = PASS, 1 = FAIL, 2 = 환경 문제(arecord/numpy 없음)
"""
import argparse
import math
import re
import shutil
import subprocess
import sys
import time
import wave

RATE = 48000          # googlevoicehat 오버레이는 48kHz 고정
CHANNELS = 1
SAMPLE_BYTES = 4      # S32_LE (SPH0645는 18bit 데이터를 32bit 프레임에 왼쪽 정렬)


def find_card():
    """arecord -l 에서 I2S 마이크 카드 번호를 찾는다."""
    out = subprocess.run(["arecord", "-l"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        m = re.match(r"card (\d+):", line)
        if m and re.search(r"googlevoice|voicehat|i2s", line, re.I):
            return int(m.group(1)), out
    return None, out


def db(x):
    return 20.0 * math.log10(max(x, 1e-12))


def bar(level_db, width=40, floor=-90.0):
    frac = (level_db - floor) / (0.0 - floor)
    n = int(max(0.0, min(1.0, frac)) * width)
    return "█" * n + "·" * (width - n)


def main():
    ap = argparse.ArgumentParser(description="I2S 마이크 테스트")
    ap.add_argument("--duration", type=float, default=5.0, help="측정 시간(초), 기본 5")
    ap.add_argument("--card", type=int, default=None, help="ALSA 카드 번호 (기본: 자동 탐지)")
    ap.add_argument("--save", metavar="FILE.wav", default=None, help="녹음을 WAV로 저장")
    ap.add_argument("--require-sound", action="store_true",
                    help="소리 크기 변화가 감지되지 않으면 FAIL 처리")
    args = ap.parse_args()

    if shutil.which("arecord") is None:
        print("[ERROR] arecord가 없습니다: sudo apt install alsa-utils")
        return 2
    try:
        import numpy as np
    except ImportError:
        print("[ERROR] numpy가 없습니다: sudo apt install python3-numpy")
        print("        (가상환경은 --system-site-packages 옵션으로 만들어야 합니다. setup.sh 사용 권장)")
        return 2

    print("=== I2S 마이크 테스트 ===")

    card = args.card
    if card is None:
        card, listing = find_card()
        if card is None:
            print("[FAIL] I2S 마이크 카드를 찾지 못했습니다. `arecord -l` 출력:")
            print(listing or "(비어 있음)")
            print("       - /boot/firmware/config.txt 에 dtoverlay=googlevoicehat-soundcard 가 있는지")
            print("       - 설정 후 재부팅했는지 확인하세요.")
            return 1
    print(f"[OK] 마이크 카드 번호: {card}")

    cmd = ["arecord", "-q", "-D", f"plughw:{card}", "-c", str(CHANNELS),
           "-r", str(RATE), "-f", "S32_LE", "-t", "raw"]
    print(f"\n{args.duration:.0f}초 동안 녹음합니다. 도중에 박수를 치거나 말을 해 보세요.\n")

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    chunk_frames = RATE // 4                      # 0.25초 단위
    chunk_bytes = chunk_frames * SAMPLE_BYTES * CHANNELS
    total_bytes = int(args.duration * RATE) * SAMPLE_BYTES * CHANNELS

    chunks = []
    levels = []
    received = 0
    t0 = time.time()
    try:
        while received < total_bytes and time.time() - t0 < args.duration + 5:
            data = proc.stdout.read(min(chunk_bytes, total_bytes - received))
            if not data:
                break
            data = data[: (len(data) // 4) * 4]
            if not data:
                continue
            received += len(data)
            x = np.frombuffer(data, dtype="<i4").astype(np.float64) / 2147483648.0
            chunks.append(np.frombuffer(data, dtype="<i4").copy())
            ac = x - x.mean()
            rms = float(np.sqrt(np.mean(ac * ac)))
            peak = float(np.max(np.abs(ac)))
            levels.append(db(rms))
            print(f"\r{bar(db(rms))} RMS {db(rms):6.1f} dBFS  peak {db(peak):6.1f} dBFS ", end="", flush=True)
    finally:
        proc.terminate()
        try:
            err = proc.stderr.read().decode(errors="replace").strip()
        except Exception:
            err = ""
        proc.wait(timeout=3)
    print("\n")

    if received == 0:
        print("[FAIL] 마이크에서 데이터를 받지 못했습니다.")
        if err:
            print("       arecord 오류:", err)
        return 1

    samples = np.concatenate(chunks)
    x = samples.astype(np.float64) / 2147483648.0
    mean = float(x.mean())
    ac = x - mean
    rms_db = db(float(np.sqrt(np.mean(ac * ac))))
    peak_db = db(float(np.max(np.abs(ac))))
    unique = int(np.unique(samples).size)
    level_span = (max(levels) - min(levels)) if levels else 0.0

    if args.save:
        with wave.open(args.save, "wb") as wf:
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(SAMPLE_BYTES)
            wf.setframerate(RATE)
            wf.writeframes(samples.astype("<i4").tobytes())
        print(f"[INFO] 저장됨: {args.save}")

    print("--- 결과 ---")
    results = []

    expected = int(args.duration * RATE) * SAMPLE_BYTES
    ok = received >= expected * 0.9
    results.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] 데이터 수신량: {received / expected * 100:.0f}% ({received} bytes)")

    ok = unique > 16 and rms_db > -100.0
    results.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] 신호 존재: 서로 다른 값 {unique}개, AC RMS {rms_db:.1f} dBFS "
          f"{'' if ok else '(전부 0이거나 값이 고정 -> DOUT/BCLK/LRCL 배선, SEL 핀 확인)'}")

    print(f"[INFO] 최대 피크 {peak_db:.1f} dBFS, DC 오프셋 {mean:+.5f} (SPH0645는 DC 오프셋이 큰 편이라 정상)")

    reacts = level_span >= 6.0
    if reacts:
        print(f"[PASS] 소리 변화 감지: 구간별 레벨 차 {level_span:.1f} dB")
    else:
        msg = f"소리 변화가 거의 없음 (레벨 차 {level_span:.1f} dB) - 녹음 중 박수를 치면서 다시 해 보세요"
        if args.require_sound:
            results.append(False)
            print(f"[FAIL] {msg}")
        else:
            print(f"[WARN] {msg}")

    passed = all(results)
    print("\n전체 결과:", "PASS ✅" if passed else "FAIL ❌")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
