#!/usr/bin/env python3
"""eye-pi 센서 대시보드 서버.

Pi에서 실행하면 IMU(BNO086)와 I2S 마이크 값을 읽어 웹 브라우저로 실시간 전송합니다.
같은 네트워크의 Mac 브라우저에서 http://eye-pi-1.local:8000 으로 접속하세요.

    python server.py                 # 실제 센서
    python server.py --mock          # 센서 없이 가짜 데이터로 화면만 확인 (Mac에서도 실행 가능)
    python server.py --port 8080     # 포트 변경

표준 라이브러리만으로 동작합니다. numpy가 있으면 마이크 계산에 사용합니다.
"""
import argparse
import array
import codecs
import collections
import importlib
import json
import math
import os
import random
import re
import select
import shutil
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

try:
    import numpy as np
except Exception:  # numpy가 없어도 동작 (느린 순수 파이썬 계산으로 대체)
    np = None

HERE = Path(__file__).resolve().parent
WEB_DIR = HERE / "web"
HOST = socket.gethostname()

RATE = 48000                      # googlevoicehat 오버레이는 48kHz 고정
SAMPLE_BYTES = 4                  # S32_LE
CHUNK_FRAMES = RATE // 10         # 0.1초 단위로 레벨 계산
CHUNK_BYTES = CHUNK_FRAMES * SAMPLE_BYTES
FULL_SCALE = 2147483648.0
FLOOR_DB = -120.0


# ---------------------------------------------------------------- 수학 도우미
def db(x):
    return max(FLOOR_DB, 20.0 * math.log10(max(x, 1e-12)))


def quat_to_euler(x, y, z, w):
    """쿼터니언 -> (roll, pitch, yaw) 도(degree)."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def euler_to_quat(roll, pitch, yaw):
    """라디안 오일러각 -> 쿼터니언 (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def world_to_body(q, v):
    """월드 좌표 벡터를 센서(body) 좌표로 변환 (R^T v)."""
    x, y, z, w = q
    r = ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
         (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
         (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))
    return tuple(sum(r[i][j] * v[i] for i in range(3)) for j in range(3))


# ---------------------------------------------------------------- 이벤트 버스
class Sub:
    """브라우저 한 개(SSE 연결 한 개)에 대응하는 구독자."""

    def __init__(self):
        self.lock = threading.Lock()
        self.events = collections.deque(maxlen=1000)
        self.state = None
        self.wake = threading.Event()

    def push(self, event, data):
        with self.lock:
            self.events.append((event, data))
        self.wake.set()

    def set_state(self, text):
        with self.lock:
            self.state = text
        self.wake.set()

    def take(self):
        with self.lock:
            events = list(self.events)
            self.events.clear()
            state, self.state = self.state, None
        return events, state


class Bus:
    def __init__(self):
        self.lock = threading.Lock()
        self.subs = set()
        self.logs = collections.deque(maxlen=300)
        self.seq = 0

    def subscribe(self):
        sub = Sub()
        with self.lock:
            for entry in self.logs:           # 접속하면 최근 로그를 먼저 보여줌
                sub.events.append(("log", entry))
            self.subs.add(sub)
        return sub

    def unsubscribe(self, sub):
        with self.lock:
            self.subs.discard(sub)

    def active(self):
        with self.lock:
            return list(self.subs)

    def emit(self, event, data):
        for sub in self.active():
            sub.push(event, data)

    def log(self, msg, level="info"):
        with self.lock:
            self.seq += 1
            entry = {"id": self.seq, "t": time.strftime("%H:%M:%S"), "level": level, "msg": msg}
            self.logs.append(entry)
            subs = list(self.subs)
        print(f"[{entry['t']}] {msg}", flush=True)
        for sub in subs:
            sub.push("log", entry)


BUS = Bus()


class Shared:
    """센서 스레드들이 기록하고 브라우저로 내보낼 최신 값."""

    def __init__(self):
        self.lock = threading.Lock()
        self.imu = {"status": "init", "msg": "", "addr": None, "i2c_scan": [],
                    "q": [0.0, 0.0, 0.0, 1.0], "accel": [0.0] * 3, "gyro": [0.0] * 3,
                    "mag": [0.0] * 3, "rate": 0.0, "errors": 0, "calib": None, "ts": 0.0}
        self.mic = {"status": "init", "msg": "", "card": None, "rms_db": FLOOR_DB,
                    "peak_db": FLOOR_DB, "rate": 0.0, "ts": 0.0}


S = Shared()


# ---------------------------------------------------------------- 센서 읽기 스레드
class Reader(threading.Thread):
    """센서 하나를 계속 읽는 스레드. 점검 작업이 센서를 써야 할 때는 pause()로 잠시 놓아준다."""
    store_name = ""
    label = ""

    def __init__(self):
        super().__init__(daemon=True, name=self.store_name)
        self.paused = threading.Event()
        self.idle = threading.Event()
        self._noted = None

    def set(self, **kw):
        with S.lock:
            getattr(S, self.store_name).update(kw)

    def note(self, text, level="info"):
        """같은 메시지를 반복해서 로그에 남기지 않는다."""
        if text != self._noted:
            self._noted = text
            BUS.log(text, level)

    def interrupt(self):
        pass

    def pause(self, timeout=6.0):
        self.paused.set()
        self.interrupt()
        if not self.idle.wait(timeout):
            BUS.log(f"{self.label}을(를) 멈추는 데 시간이 걸리고 있어요", "warn")

    def resume(self):
        self._noted = None
        self.idle.clear()
        self.paused.clear()

    def session(self):
        raise NotImplementedError

    def run(self):
        while True:
            if self.paused.is_set():
                self.set(status="paused", msg="점검 실행 중이라 잠시 멈춤")
                self.idle.set()
                time.sleep(0.1)
                continue
            try:
                self.session()
            except Exception as e:  # noqa: BLE001 - 어떤 오류든 화면에 보여주고 재시도
                if self.paused.is_set():
                    continue
                msg = f"{type(e).__name__}: {e}"
                self.set(status="error", msg=msg)
                self.note(f"{self.label} 오류: {msg}", "err")
                self.paused.wait(3.0)
                continue
            self.paused.wait(0.2)


class ImuReader(Reader):
    store_name = "imu"
    label = "IMU"

    def __init__(self, use_game_rotation=False):
        super().__init__()
        self.use_game = use_game_rotation

    def session(self):
        try:
            import board
            import busio
            lib = importlib.import_module("adafruit_bno08x")
            from adafruit_bno08x.i2c import BNO08X_I2C
        except Exception as e:  # noqa: BLE001
            msg = f"라이브러리를 불러올 수 없음: {e}"
            self.set(status="error", msg=msg)
            self.note(f"IMU {msg} (setup.sh 실행 여부와 .venv 확인)", "err")
            self.paused.wait(10.0)
            return

        i2c = busio.I2C(board.SCL, board.SDA)
        try:
            while not i2c.try_lock():
                time.sleep(0.01)
            try:
                found = i2c.scan()
            finally:
                i2c.unlock()
            self.set(i2c_scan=[hex(a) for a in found])

            candidates = [a for a in (0x4B, 0x4A) if a in found]
            if not candidates:
                self.set(status="not_found", addr=None,
                         msg="I2C에서 BNO086(0x4B/0x4A)을 찾지 못함 - 배선을 확인하세요")
                seen = ", ".join(hex(a) for a in found) or "없음"
                self.note(f"IMU를 찾지 못했어요 (I2C 장치: {seen}). 3V3/GND/SDA/SCL 배선을 확인하세요", "warn")
                self.paused.wait(3.0)
                return

            bno, addr = None, None
            for a in candidates:
                for attempt in range(3):
                    try:
                        bno = BNO08X_I2C(i2c, address=a)
                        addr = a
                        break
                    except Exception as e:  # noqa: BLE001
                        BUS.log(f"IMU 초기화 재시도 {attempt + 1}/3 (주소 {hex(a)}): {e}", "warn")
                        time.sleep(0.5)
                if bno:
                    break
            if bno is None:
                raise RuntimeError("BNO086 초기화 실패 - 센서 전원을 껐다 켜 보세요")

            quat_feature = "BNO_REPORT_GAME_ROTATION_VECTOR" if self.use_game else "BNO_REPORT_ROTATION_VECTOR"
            for feature in ("BNO_REPORT_ACCELEROMETER", "BNO_REPORT_GYROSCOPE",
                            "BNO_REPORT_MAGNETOMETER", quat_feature):
                bno.enable_feature(getattr(lib, feature))
            time.sleep(0.3)

            self.set(status="ok", msg="", addr=hex(addr), errors=0)
            self._noted = None
            kind = "게임 회전벡터(지자기 미사용)" if self.use_game else "회전벡터(9축 융합)"
            BUS.log(f"IMU 연결됨 (주소 {hex(addr)}, {kind})", "ok")
            self.loop(bno)
        finally:
            try:
                i2c.deinit()
            except Exception:  # noqa: BLE001
                pass

    def loop(self, bno):
        quat_attr = "game_quaternion" if self.use_game else "quaternion"
        last_ok = time.time()
        last_sig = None
        changes = 0
        t_rate = time.time()
        last_cal = 0.0
        errors = 0
        while not self.paused.is_set():
            try:
                qi, qj, qk, qr = getattr(bno, quat_attr)
                accel = bno.acceleration
                gyro = bno.gyro
                mag = bno.magnetic
            except (KeyError, RuntimeError, OSError, TypeError):
                errors += 1
                self.set(errors=errors)
                if time.time() - last_ok > 3.0:
                    raise RuntimeError("3초 동안 IMU 데이터를 받지 못함")
                time.sleep(0.05)
                continue

            now = time.time()
            last_ok = now
            sig = (qi, qj, qk, qr) + tuple(accel)
            if sig != last_sig:
                changes += 1
                last_sig = sig
            update = {"q": [qi, qj, qk, qr], "accel": list(accel), "gyro": list(gyro),
                      "mag": list(mag), "ts": now}
            if now - t_rate >= 1.0:
                update["rate"] = changes / (now - t_rate)
                changes, t_rate = 0, now
            if now - last_cal >= 3.0:
                last_cal = now
                try:
                    update["calib"] = int(bno.calibration_status)
                except Exception:  # noqa: BLE001 - 라이브러리 버전에 따라 없을 수 있음
                    update["calib"] = None
            self.set(**update)
            time.sleep(0.025)


def find_card():
    """arecord -l 에서 I2S 마이크 카드 번호를 찾는다."""
    try:
        out = subprocess.run(["arecord", "-l"], capture_output=True, text=True, timeout=5).stdout
    except Exception:  # noqa: BLE001
        return None
    for line in out.splitlines():
        m = re.match(r"card (\d+):", line)
        if m and re.search(r"googlevoice|voicehat|i2s", line, re.I):
            return int(m.group(1))
    return None


def chunk_levels(chunk):
    """32비트 리틀엔디언 PCM 한 덩어리 -> (RMS dBFS, 피크 dBFS)."""
    if np is not None:
        x = np.frombuffer(chunk, dtype="<i4").astype(np.float64) / FULL_SCALE
        ac = x - x.mean()
        return db(float(np.sqrt(np.mean(ac * ac)))), db(float(np.max(np.abs(ac))))
    arr = array.array("i")
    arr.frombytes(chunk)
    if sys.byteorder == "big":
        arr.byteswap()
    n = len(arr)
    mean = sum(arr) / n
    ss, peak = 0.0, 0.0
    for v in arr:
        d = v - mean
        ss += d * d
        if abs(d) > peak:
            peak = abs(d)
    return db(math.sqrt(ss / n) / FULL_SCALE), db(peak / FULL_SCALE)


class MicReader(Reader):
    store_name = "mic"
    label = "마이크"

    def __init__(self, card=None):
        super().__init__()
        self.card = card
        self._proc = None

    def interrupt(self):
        proc = self._proc
        if proc is not None:
            try:
                proc.terminate()
            except Exception:  # noqa: BLE001
                pass

    def session(self):
        if shutil.which("arecord") is None:
            self.set(status="error", msg="arecord가 없음 (sudo apt install alsa-utils)")
            self.note("마이크: arecord가 없어요. sudo apt install alsa-utils", "err")
            self.paused.wait(10.0)
            return
        card = self.card if self.card is not None else find_card()
        if card is None:
            self.set(status="not_found", card=None,
                     msg="I2S 마이크 카드를 찾지 못함 (config.txt의 dtoverlay와 재부팅 확인)")
            self.note("마이크 카드를 찾지 못했어요. dtoverlay=googlevoicehat-soundcard 설정과 재부팅을 확인하세요", "warn")
            self.paused.wait(3.0)
            return

        cmd = ["arecord", "-q", "-D", f"plughw:{card}", "-c", "1", "-r", str(RATE),
               "-f", "S32_LE", "-t", "raw"]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self._proc = proc
        try:
            fd = proc.stdout.fileno()
            buf = bytearray()
            stalls = 0
            chunks = 0
            t_rate = time.time()
            self.set(status="ok", msg="", card=card)
            self._noted = None
            BUS.log(f"마이크 연결됨 (카드 {card}, 48kHz)", "ok")
            while not self.paused.is_set():
                ready, _, _ = select.select([fd], [], [], 1.0)
                if not ready:
                    if proc.poll() is not None:
                        break
                    stalls += 1
                    if stalls >= 3:
                        raise RuntimeError("마이크 데이터가 3초 동안 들어오지 않음")
                    continue
                stalls = 0
                data = os.read(fd, 65536)
                if not data:
                    break
                buf.extend(data)
                while len(buf) >= CHUNK_BYTES:
                    chunk = bytes(buf[:CHUNK_BYTES])
                    del buf[:CHUNK_BYTES]
                    rms_db, peak_db = chunk_levels(chunk)
                    chunks += 1
                    now = time.time()
                    update = {"rms_db": rms_db, "peak_db": peak_db, "ts": now}
                    if now - t_rate >= 1.0:
                        update["rate"] = chunks / (now - t_rate)
                        chunks, t_rate = 0, now
                    self.set(**update)
            if not self.paused.is_set():
                try:
                    err = proc.stderr.read().decode(errors="replace").strip()
                except Exception:  # noqa: BLE001
                    err = ""
                raise RuntimeError(f"arecord 종료: {err or '이유를 알 수 없음'}")
        finally:
            self._proc = None
            try:
                proc.terminate()
                proc.wait(2)
            except Exception:  # noqa: BLE001
                try:
                    proc.kill()
                except Exception:  # noqa: BLE001
                    pass


# ---------------------------------------------------------------- 모의(mock) 센서
class MockImu(Reader):
    store_name = "imu"
    label = "IMU(모의)"

    def session(self):
        BUS.log("[mock] 가짜 IMU 데이터를 만듭니다 (센서 없이 화면 확인용)", "info")
        self.set(status="ok", msg="모의 데이터", addr="0x4b (mock)", i2c_scan=["0x4b"], errors=0, calib=3)
        t0 = time.time()
        count, t_rate = 0, time.time()
        while not self.paused.is_set():
            t = time.time() - t0
            roll = math.radians(38 * math.sin(t * 0.9))
            pitch = math.radians(26 * math.sin(t * 0.6 + 1.0))
            yaw = t * 0.7
            q = euler_to_quat(roll, pitch, yaw)
            g = world_to_body(q, (0.0, 0.0, 9.81))
            m = world_to_body(q, (0.0, 22.0, -42.0))
            update = {
                "q": list(q),
                "accel": [g[i] + random.gauss(0, 0.03) for i in range(3)],
                "gyro": [math.radians(38) * 0.9 * math.cos(t * 0.9), math.radians(26) * 0.6 * math.cos(t * 0.6 + 1.0), 0.7],
                "mag": [m[i] + random.gauss(0, 0.2) for i in range(3)],
                "ts": time.time(),
            }
            count += 1
            if time.time() - t_rate >= 1.0:
                update["rate"] = count / (time.time() - t_rate)
                count, t_rate = 0, time.time()
            self.set(**update)
            time.sleep(0.05)


class MockMic(Reader):
    store_name = "mic"
    label = "마이크(모의)"

    def session(self):
        BUS.log("[mock] 가짜 마이크 레벨을 만듭니다", "info")
        self.set(status="ok", msg="모의 데이터", card="mock")
        t0 = time.time()
        count, t_rate = 0, time.time()
        while not self.paused.is_set():
            t = time.time() - t0
            phase = t % 4.0
            clap = max(0.0, 1.0 - phase / 0.5) * 48.0          # 4초마다 박수
            talk = max(0.0, math.sin(t * 1.3)) ** 2 * 22.0       # 천천히 오르내리는 말소리
            rms = -66.0 + 2.0 * math.sin(t * 3.1) + random.gauss(0, 0.6) + max(clap, talk)
            rms = min(rms, -3.0)
            update = {"rms_db": rms, "peak_db": min(rms + 9.0 + random.random() * 3.0, -0.5), "ts": time.time()}
            count += 1
            if time.time() - t_rate >= 1.0:
                update["rate"] = count / (time.time() - t_rate)
                count, t_rate = 0, time.time()
            self.set(**update)
            time.sleep(0.1)


# ---------------------------------------------------------------- 시스템 정보
class SysInfo:
    def __init__(self):
        self.cache, self.stamp = {}, 0.0

    def get(self):
        now = time.time()
        if now - self.stamp < 2.0:
            return self.cache
        info = {"host": HOST, "ip": None, "temp": None, "load": None,
                "mem_used": None, "mem_total": None, "uptime": None}
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("10.255.255.255", 1))
            info["ip"] = s.getsockname()[0]
            s.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            info["temp"] = round(int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000.0, 1)
        except Exception:  # noqa: BLE001
            pass
        try:
            info["load"] = [round(v, 2) for v in os.getloadavg()]
        except Exception:  # noqa: BLE001
            pass
        try:
            mem = {}
            for line in Path("/proc/meminfo").read_text().splitlines():
                key, val = line.split(":", 1)
                mem[key] = int(val.split()[0])
            info["mem_total"] = round(mem["MemTotal"] / 1024)
            info["mem_used"] = round((mem["MemTotal"] - mem["MemAvailable"]) / 1024)
        except Exception:  # noqa: BLE001
            pass
        try:
            info["uptime"] = int(float(Path("/proc/uptime").read_text().split()[0]))
        except Exception:  # noqa: BLE001
            pass
        self.cache, self.stamp = info, now
        return info


SYS = SysInfo()


# ---------------------------------------------------------------- 점검 작업 (터미널 명령을 UI에서 실행)
class LineFeed:
    """프로세스 출력을 줄 단위로 쪼갠다. '\\r'로 같은 줄을 덮어쓰는 출력(레벨 미터)도 처리."""

    def __init__(self):
        self.buf = ""
        self.live = False

    @staticmethod
    def _emit(text, live):
        BUS.emit("job", {"type": "line", "text": text, "live": live})

    def feed(self, text):
        for ch in text:
            if ch == "\r":
                if self.buf:
                    self._emit(self.buf, True)
                    self.live = True
                    self.buf = ""
            elif ch == "\n":
                if self.buf:
                    self._emit(self.buf, False)
                    self.live = False
                    self.buf = ""
                elif self.live:
                    BUS.emit("job", {"type": "final"})
                    self.live = False
                else:
                    self._emit("", False)
            else:
                self.buf += ch

    def flush(self):
        if self.buf:
            self._emit(self.buf, False)
            self.buf = ""
            self.live = False


def _i2cdetect():
    return shutil.which("i2cdetect") or "/usr/sbin/i2cdetect"


JOB_SPECS = {
    "i2c": {"label": "I2C 스캔", "show": "i2cdetect -y 1", "pause": ["imu"],
            "cmd": lambda: [_i2cdetect(), "-y", "1"]},
    "audio": {"label": "오디오 장치", "show": "arecord -l", "pause": [],
              "cmd": lambda: ["arecord", "-l"]},
    "mic": {"label": "마이크 테스트", "show": "python test_mic.py --duration 5", "pause": ["mic"],
            "cmd": lambda: [sys.executable, "-u", str(HERE / "test_mic.py"), "--duration", "5"]},
    "imu": {"label": "IMU 테스트", "show": "python test_imu.py --duration 10", "pause": ["imu"],
            "cmd": lambda: [sys.executable, "-u", str(HERE / "test_imu.py"), "--duration", "10"]},
    "all": {"label": "전체 점검", "show": "python check_all.py", "pause": ["imu", "mic"],
            "cmd": lambda: [sys.executable, "-u", str(HERE / "check_all.py")]},
}

MOCK_OUTPUT = {
    "i2c": [("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f", 0.1, False),
            ("00:                         -- -- -- -- -- -- -- -- ", 0.05, False),
            ("40: -- -- -- -- -- -- -- -- -- -- -- 4b -- -- -- -- ", 0.05, False)],
    "audio": [("**** List of CAPTURE Hardware Devices ****", 0.1, False),
              ("card 1: sndrpigooglevoi [snd_rpi_googlevoicehat_soundcar], device 0: Google voiceHAT SoundCard HiFi", 0.05, False)],
    "mic": [("=== I2S 마이크 테스트 (모의) ===", 0.2, False)]
           + [(f"RMS {-60 + i * 2:6.1f} dBFS", 0.25, True) for i in range(8)]
           + [("[PASS] 데이터 수신량: 100%", 0.1, False), ("[PASS] 소리 변화 감지", 0.1, False),
              ("전체 결과: PASS ✅", 0.1, False)],
    "imu": [("=== BNO086 IMU 테스트 (모의) ===", 0.2, False),
            ("I2C 스캔 결과: ['0x4b']", 0.2, False),
            ("[OK] BNO086 초기화 성공 (주소 0x4b)", 0.4, False),
            ("[PASS] 가속도 크기 평균 9.81 m/s²", 0.5, False),
            ("[PASS] 쿼터니언 크기 평균 1.000", 0.1, False),
            ("전체 결과: PASS ✅", 0.1, False)],
}
MOCK_OUTPUT["all"] = MOCK_OUTPUT["mic"] + MOCK_OUTPUT["imu"]


class JobRunner:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = None
        self.last = None
        self.readers = []
        self.mock = False

    def info(self):
        with self.lock:
            return {"running": self.running, "last": self.last}

    def start(self, name):
        if name not in JOB_SPECS:
            return False, "알 수 없는 작업입니다"
        with self.lock:
            if self.running:
                return False, "다른 작업이 실행 중입니다"
            self.running = name
        threading.Thread(target=self._run, args=(name,), daemon=True).start()
        return True, "시작했어요"

    def _run(self, name):
        spec = JOB_SPECS[name]
        paused = [r for r in self.readers if r.store_name in spec["pause"]]
        code = None
        BUS.emit("job", {"type": "start", "name": name, "label": spec["label"], "cmd": spec["show"]})
        try:
            for reader in paused:           # 센서를 한 프로세스만 쓸 수 있어서 잠시 멈춘다
                reader.pause()
            code = self._run_mock(name) if self.mock else self._run_process(spec)
        except Exception as e:  # noqa: BLE001
            BUS.emit("job", {"type": "line", "text": f"실행 실패: {e}", "live": False})
            code = -1
        finally:
            for reader in paused:
                reader.resume()
            with self.lock:
                self.running = None
                self.last = {"name": name, "code": code}
            BUS.emit("job", {"type": "done", "name": name, "code": code})

    def _run_mock(self, name):
        feed = LineFeed()
        for text, delay, live in MOCK_OUTPUT[name]:
            time.sleep(delay)
            feed.feed(text + ("\r" if live else "\n"))
        feed.feed("\n")
        feed.flush()
        return 0

    def _run_process(self, spec):
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        try:
            proc = subprocess.Popen(spec["cmd"](), cwd=str(HERE), env=env,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        except FileNotFoundError as e:
            BUS.emit("job", {"type": "line", "text": f"명령을 찾을 수 없음: {e.filename}", "live": False})
            return 127
        decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
        feed = LineFeed()
        fd = proc.stdout.fileno()
        while True:
            data = os.read(fd, 4096)
            if not data:
                break
            feed.feed(decoder.decode(data))
        feed.feed(decoder.decode(b"", final=True))
        feed.flush()
        return proc.wait()


JOB = JobRunner()


# ---------------------------------------------------------------- HTTP
def build_snapshot(mock=False):
    now = time.time()
    with S.lock:
        imu, mic = dict(S.imu), dict(S.mic)
    imu["age"] = round(now - imu["ts"], 2) if imu["ts"] else None
    mic["age"] = round(now - mic["ts"], 2) if mic["ts"] else None
    imu["q"] = [round(v, 5) for v in imu["q"]]
    for key in ("accel", "gyro", "mag"):
        imu[key] = [round(v, 3) for v in imu[key]]
    imu["rate"] = round(imu["rate"], 1)
    mic["rms_db"] = round(mic["rms_db"], 1)
    mic["peak_db"] = round(mic["peak_db"], 1)
    mic["rate"] = round(mic["rate"], 1)
    return {"t": round(now, 3), "mock": mock, "imu": imu, "mic": mic,
            "sys": SYS.get(), "job": JOB.info()}


class Handler(BaseHTTPRequestHandler):
    server_version = "eye-pi-dashboard"
    mock = False

    def log_message(self, fmt, *args):  # 접속 로그는 조용히
        pass

    def _send(self, code, body=b"", ctype="text/plain; charset=utf-8"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code, obj):
        self._send(code, json.dumps(obj, ensure_ascii=False).encode(), "application/json; charset=utf-8")

    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            try:
                self._send(200, (WEB_DIR / "index.html").read_bytes(), "text/html; charset=utf-8")
            except OSError:
                self._send(500, "web/index.html 파일이 없습니다".encode())
        elif path == "/api/state":
            self._json(200, build_snapshot(self.mock))
        elif path == "/api/stream":
            self._stream()
        elif path == "/favicon.ico":
            self._send(204)
        else:
            self._send(404, b"not found")

    def do_POST(self):
        path = urlparse(self.path).path
        origin = self.headers.get("Origin")
        if origin and urlparse(origin).netloc != self.headers.get("Host"):
            self._json(403, {"ok": False, "msg": "다른 사이트에서 보낸 요청은 막습니다"})
            return
        if path != "/api/run":
            self._json(404, {"ok": False, "msg": "not found"})
            return
        try:
            length = min(int(self.headers.get("Content-Length", "0")), 1024)
            body = json.loads(self.rfile.read(length) or b"{}")
            name = str(body.get("job", ""))
        except Exception:  # noqa: BLE001
            self._json(400, {"ok": False, "msg": "요청 형식이 올바르지 않습니다"})
            return
        ok, msg = JOB.start(name)
        self._json(200 if ok else 409, {"ok": ok, "msg": msg})

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        sub = BUS.subscribe()
        try:
            self.wfile.write(b"retry: 2000\n\n")
            self.wfile.flush()
            while True:
                sub.wake.wait(15.0)
                sub.wake.clear()
                events, state = sub.take()
                out = []
                for event, data in events:
                    out.append(f"event: {event}\ndata: {json.dumps(data, ensure_ascii=False)}\n\n")
                if state:
                    out.append(f"data: {state}\n\n")
                if not out:
                    out.append(": ping\n\n")
                self.wfile.write("".join(out).encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            BUS.unsubscribe(sub)


def publisher(mock):
    """초당 20번 현재 값을 모아 접속한 브라우저들에게 보낸다."""
    while True:
        subs = BUS.active()
        if subs:
            text = json.dumps(build_snapshot(mock), ensure_ascii=False, separators=(",", ":"))
            for sub in subs:
                sub.set_state(text)
        time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser(description="eye-pi 센서 대시보드 서버")
    ap.add_argument("--host", default="0.0.0.0", help="바인드 주소 (기본: 모든 네트워크)")
    ap.add_argument("--port", type=int, default=8000, help="포트 (기본: 8000)")
    ap.add_argument("--mock", action="store_true", help="센서 없이 가짜 데이터로 실행")
    ap.add_argument("--card", type=int, default=None, help="ALSA 마이크 카드 번호 (기본: 자동 탐지)")
    ap.add_argument("--game-rotation", action="store_true",
                    help="지자기 없이 회전벡터 계산 (실내에서 방향이 흔들릴 때)")
    ap.add_argument("--no-imu", action="store_true", help="IMU 읽기 끄기")
    ap.add_argument("--no-mic", action="store_true", help="마이크 읽기 끄기")
    args = ap.parse_args()

    readers = []
    if not args.no_imu:
        readers.append(MockImu() if args.mock else ImuReader(args.game_rotation))
    if not args.no_mic:
        readers.append(MockMic() if args.mock else MicReader(args.card))
    JOB.readers = readers
    JOB.mock = args.mock
    Handler.mock = args.mock

    for reader in readers:
        reader.start()
    threading.Thread(target=publisher, args=(args.mock,), daemon=True).start()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    ip = SYS.get().get("ip")
    print("=" * 56)
    print(" eye-pi 센서 대시보드" + (" (모의 데이터)" if args.mock else ""))
    print(f"   http://{HOST}.local:{args.port}")
    if ip:
        print(f"   http://{ip}:{args.port}")
    print(" 종료: Ctrl+C")
    print("=" * 56, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n종료합니다")
    finally:
        for reader in readers:
            reader.pause(timeout=2.0)
        server.server_close()


if __name__ == "__main__":
    main()
