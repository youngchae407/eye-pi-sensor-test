#!/usr/bin/env python3
"""하드웨어 진단 화면 서버 (배선/전원 문제를 실시간으로 보기).

Pi 에서 실행하고 Mac 브라우저로 http://eye-pi-1.local:8081 에 접속하세요.

    python tools/hwdebug.py              # Pi 에서 (가상환경 없이도 동작, 표준 라이브러리만 사용)
    python tools/hwdebug.py --mock       # Mac 에서 화면만 확인
    python tools/hwdebug.py --shunt 0.1  # INA219 전류 센서의 션트 저항(Ω)

보여 주는 것:
- I2C(SDA/SCL), IMU RST/INT, 마이크 핀의 실제 전압 상태(HIGH/LOW)와 변화 기록
- Pi 전원 상태(저전압 경고, 코어 전압, 온도)
- INA219 전류 센서를 연결했다면 IMU 전류(mA) 그래프 (없으면 멀티미터로 재고 값을 입력)
- 실험 버튼: I2C 주소 검색, 풀업/풀다운으로 '누가 선을 붙잡고 있나' 확인, 버스 복구, RST 리셋

Pi 의 전원 레일 전류는 Pi Zero 에서 소프트웨어로 읽을 수 없습니다. 실제 전류는 INA219 같은
전류 센서나 멀티미터가 있어야 잴 수 있습니다.

센서를 쓰는 server.py 와 동시에 실행하면 핀 실험이 서로 방해할 수 있으니, 실험 전에는 대시보드를 멈추세요.
"""
import argparse
import fcntl
import json
import math
import os
import random
import re
import shutil
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

HERE = Path(__file__).resolve().parent.parent
PAGE = HERE / "web" / "hwdebug.html"
I2C_SLAVE = 0x0703

# BCM 번호 -> (이름, 물리 핀). README 의 실제 배선 기준
PINS = {
    2: ("I2C SDA", 3),
    3: ("I2C SCL", 5),
    24: ("IMU RST", 18),
    17: ("IMU INT", 11),
    18: ("마이크 BCLK", 12),
    19: ("마이크 LRCL", 35),
    20: ("마이크 DOUT", 38),
}
FUNC_NAMES = {"a0": "I2C", "ip": "입력", "op": "출력", "a5": "ALT5", "a3": "ALT3", "a2": "ALT2",
              "a1": "ALT1", "a4": "ALT4", "no": "꺼짐"}
PULL_NAMES = {"pu": "풀업", "pd": "풀다운", "pn": "없음", "--": "없음"}
THROTTLE_BITS = {
    0: "지금 저전압", 1: "지금 클럭 제한", 2: "지금 스로틀링", 3: "지금 온도 제한",
    16: "부팅 후 저전압 있었음", 17: "부팅 후 클럭 제한 있었음", 18: "부팅 후 스로틀링 있었음",
    19: "부팅 후 온도 제한 있었음",
}


def run(cmd, timeout=3.0):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return (r.stdout or "") + (r.stderr or "")
    except Exception as e:  # noqa: BLE001
        return f"ERR {e}"


# ---------------------------------------------------------------- 하드웨어 접근
class RealHw:
    mock = False

    def __init__(self, bus, reset_bcm):
        self.bus = bus
        self.reset_bcm = reset_bcm
        self.has_pinctrl = shutil.which("pinctrl") is not None
        self.model = Path("/proc/device-tree/model").read_text(errors="ignore").strip("\x00\n ") \
            if Path("/proc/device-tree/model").exists() else "알 수 없음"

    # 핀 ---------------------------------------------------------------
    def pins(self):
        if not self.has_pinctrl:
            return {}
        out = run(["pinctrl", "get", ",".join(str(p) for p in PINS)])
        res = {}
        for line in out.splitlines():
            m = re.match(r"\s*(\d+):\s*(\S+)\s+(\S+)\s*\|\s*(hi|lo)", line)
            if m:
                res[int(m.group(1))] = {"func": m.group(2), "pull": m.group(3), "level": m.group(4)}
        return res

    def pinctrl(self, *args):
        return run(["pinctrl", "set", *[str(a) for a in args]])

    # 전원 -------------------------------------------------------------
    def power(self):
        info = {}
        m = re.search(r"0x([0-9a-fA-F]+)", run(["vcgencmd", "get_throttled"]))
        if m:
            info["throttled"] = int(m.group(1), 16)
        m = re.search(r"([\d.]+)V", run(["vcgencmd", "measure_volts", "core"]))
        if m:
            info["core_v"] = float(m.group(1))
        try:
            info["temp_c"] = int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000
        except Exception:  # noqa: BLE001
            pass
        for hw in Path("/sys/class/hwmon").glob("hwmon*"):
            try:
                if (hw / "name").read_text().strip() == "rpi_volt":
                    info["uv_alarm"] = int((hw / "in1_lcrit_alarm").read_text().strip())
            except Exception:  # noqa: BLE001
                pass
        try:
            info["uptime_s"] = float(Path("/proc/uptime").read_text().split()[0])
        except Exception:  # noqa: BLE001
            pass
        return info

    def other_users(self):
        """센서를 쓰는 다른 프로세스 (server.py, test_imu.py 등)."""
        out = run(["pgrep", "-af", "server.py|test_imu.py|imu_diag.py|check_all.py"])
        return [l for l in out.splitlines() if l.strip() and "hwdebug" not in l and not l.startswith("ERR")]

    # I2C --------------------------------------------------------------
    def probe(self, addr):
        """주소에 1바이트 읽기를 시도. True=응답, False=응답 없음"""
        fd = os.open(f"/dev/i2c-{self.bus}", os.O_RDWR)
        try:
            fcntl.ioctl(fd, I2C_SLAVE, addr)
            os.read(fd, 1)
            return True
        except OSError:
            return False
        finally:
            os.close(fd)

    def ina219(self, addr, shunt_ohm):
        """INA219: (버스 전압 V, 전류 mA). 레지스터 1=션트 전압(10µV/LSB), 2=버스 전압(4mV/LSB, >>3)"""
        fd = os.open(f"/dev/i2c-{self.bus}", os.O_RDWR)
        try:
            fcntl.ioctl(fd, I2C_SLAVE, addr)

            def reg(r):
                os.write(fd, bytes([r]))
                hi, lo = os.read(fd, 2)
                return (hi << 8) | lo

            shunt = reg(0x01)
            if shunt & 0x8000:
                shunt -= 0x10000
            bus_v = (reg(0x02) >> 3) * 0.004
            return bus_v, shunt * 10e-6 / shunt_ohm * 1000
        finally:
            os.close(fd)


class MockHw:
    """Mac 에서 화면을 확인하기 위한 가짜 하드웨어. 'I2C 선이 LOW 로 붙잡힌' 오늘 상황을 흉내 낸다."""
    mock = True
    model = "Raspberry Pi Zero 2 W Rev 1.0 (mock)"

    def __init__(self, bus, reset_bcm):
        self.reset_bcm = reset_bcm
        self.state = {2: ["a0", "--", "lo"], 3: ["a0", "--", "lo"], 24: ["ip", "--", "hi"],
                      17: ["ip", "--", "hi"], 18: ["a0", "pd", "lo"], 19: ["a0", "pd", "lo"],
                      20: ["a0", "pd", "lo"]}
        self.stuck = True
        self.t0 = time.time()

    def pins(self):
        res = {}
        for p, (f, pull, lvl) in self.state.items():
            if p in (18, 19) and random.random() < 0.5:
                lvl = "hi" if lvl == "lo" else "lo"
            if p in (2, 3) and not self.stuck:
                lvl = "hi"
            res[p] = {"func": f, "pull": pull, "level": lvl}
        return res

    def pinctrl(self, *args):
        pins = [int(x) for x in str(args[0]).split(",")]
        rest = [str(a) for a in args[1:]]
        for p in pins:
            st = self.state[p]
            for a in rest:
                if a in ("ip", "op", "a0"):
                    st[0] = a
                elif a in ("pu", "pd", "pn"):
                    st[1] = a
                elif a in ("dl", "dh"):
                    st[2] = "lo" if a == "dl" else "hi"
            if p == 24 and "dl" in rest:
                self.stuck = False  # mock: RST 리셋으로 버스가 풀리는 경우를 흉내
            if p in (2, 3):
                st[2] = "lo" if self.stuck else ("hi" if st[1] != "pd" else "hi")
            elif st[0] == "ip":
                st[2] = "hi"  # 센서 보드의 풀업이 이김
        return ""

    def power(self):
        return {"throttled": 0x0, "core_v": 1.2625, "temp_c": 40 + random.random(),
                "uv_alarm": 0, "uptime_s": time.time() - self.t0 + 9000}

    def other_users(self):
        return []

    def probe(self, addr):
        time.sleep(0.004 if not self.stuck else 0.05)
        return (not self.stuck) and addr in (0x4B, 0x40)

    def ina219(self, addr, shunt_ohm):
        t = time.time()
        return 3.29 + 0.01 * math.sin(t), 11.5 + 2.5 * math.sin(t * 2.1) + random.random()


# ---------------------------------------------------------------- 상태 수집
class Monitor:
    def __init__(self, hw, shunt):
        self.hw = hw
        self.shunt = shunt
        self.lock = threading.Lock()       # 핀/I2C 를 건드리는 실험은 한 번에 하나
        self.cv = threading.Condition()
        self.seq = 0
        self.snapshot = {}
        self.log = deque(maxlen=200)
        self.history = {p: deque(maxlen=240) for p in PINS}  # (t, level) 0.25s * 240 = 60s
        self.current = deque(maxlen=240)
        self.ina_addr = None
        self.last_pins = {}
        self.busy = None
        self.add_log("진단 서버 시작" + (" (mock: 가짜 하드웨어)" if hw.mock else ""), "info")

    def add_log(self, msg, level="info"):
        self.log.append({"t": time.time(), "msg": msg, "level": level})
        self.bump()

    def bump(self):
        with self.cv:
            self.seq += 1
            self.cv.notify_all()

    def loop(self):
        tick = 0
        power, users = {}, []
        while True:
            now = time.time()
            if not self.busy:
                pins = self.hw.pins()
                for p, st in pins.items():
                    self.history[p].append((now, 1 if st["level"] == "hi" else 0))
                    old = self.last_pins.get(p)
                    if old and old["level"] != st["level"] and p in (2, 3, 17, 24):
                        name, phys = PINS[p]
                        self.add_log(f"{name}(핀{phys}) {old['level'].upper()} → {st['level'].upper()}", "info")
                self.last_pins = pins
                if self.ina_addr is not None and self.i2c_lines_ok():
                    try:
                        v, ma = self.hw.ina219(self.ina_addr, self.shunt)
                        self.current.append((now, round(ma, 2), round(v, 3)))
                    except OSError as e:
                        self.add_log(f"INA219 읽기 실패: {e}", "warn")
                        self.ina_addr = None
            if tick % 4 == 0:
                power = self.hw.power()
                users = self.hw.other_users()
            self.snapshot = {
                "t": now, "model": self.hw.model, "mock": self.hw.mock,
                "pins": {str(p): {**st, "name": PINS[p][0], "phys": PINS[p][1]} for p, st in self.last_pins.items()},
                "history": {str(p): [lvl for _, lvl in h] for p, h in self.history.items()},
                "power": power,
                "throttle_flags": [txt for bit, txt in THROTTLE_BITS.items() if power.get("throttled", 0) >> bit & 1],
                "current": list(self.current)[-240:],
                "ina_addr": self.ina_addr, "shunt": self.shunt,
                "users": users, "busy": self.busy,
                "log": list(self.log)[-60:],
            }
            self.bump()
            tick += 1
            time.sleep(0.25)

    def i2c_lines_ok(self):
        return all(self.last_pins.get(p, {}).get("level") == "hi" for p in (2, 3))

    # 실험 ---------------------------------------------------------------
    def action(self, name):
        fn = {"scan": self.act_scan, "pulltest": self.act_pulltest, "recover": self.act_recover,
              "reset": self.act_reset, "restore": self.act_restore}.get(name)
        if not fn:
            return {"ok": False, "msg": "알 수 없는 실험"}
        if not self.lock.acquire(blocking=False):
            return {"ok": False, "msg": f"다른 실험({self.busy}) 진행 중"}
        self.busy = name
        try:
            return fn()
        except Exception as e:  # noqa: BLE001
            self.add_log(f"{name} 오류: {type(e).__name__}: {e}", "err")
            return {"ok": False, "msg": str(e)}
        finally:
            self.busy = None
            self.lock.release()
            self.bump()

    def act_scan(self):
        stuck = not self.i2c_lines_ok()
        if stuck:
            addrs = [0x4A, 0x4B] + list(range(0x40, 0x46))
            self.add_log("SDA/SCL 이 LOW 라 전체 검색 대신 주요 주소만 확인합니다 (선이 붙잡히면 주소마다 오래 걸림)", "warn")
        else:
            addrs = list(range(0x08, 0x78))
        t0 = time.time()
        found = [a for a in addrs if self.hw.probe(a)]
        dt = time.time() - t0
        names = {0x4A: "BNO086(대체)", 0x4B: "BNO086", **{a: "INA219?" for a in range(0x40, 0x46)}}
        desc = ", ".join(f"{hex(a)} {names.get(a, '')}".strip() for a in found) or "없음"
        self.add_log(f"I2C 검색 ({len(addrs)}개 주소, {dt:.1f}초): {desc}", "ok" if found else "warn")
        ina = [a for a in found if 0x40 <= a <= 0x45]
        if ina and self.ina_addr is None:
            self.ina_addr = ina[0]
            self.current.clear()
            self.add_log(f"전류 센서(INA219 추정) {hex(ina[0])} 를 읽기 시작합니다 (션트 {self.shunt}Ω)", "ok")
        return {"ok": True, "found": [hex(a) for a in found], "seconds": round(dt, 2)}

    def act_pulltest(self):
        """각 핀을 입력으로 두고 풀업/풀다운을 걸어, 외부에서 무엇이 선을 정하는지 본다."""
        before = self.hw.pins()
        results = {}
        for p in (2, 3, 24, 17):
            self.hw.pinctrl(p, "ip", "pu")
            time.sleep(0.05)
            up = self.hw.pins().get(p, {}).get("level")
            self.hw.pinctrl(p, "ip", "pd")
            time.sleep(0.05)
            down = self.hw.pins().get(p, {}).get("level")
            self.hw.pinctrl(p, "ip", "pn")
            results[p] = (up, down)
        self._restore(before)
        msgs = []
        for p, (up, down) in results.items():
            name, phys = PINS[p]
            if up == "lo" and down == "lo":
                verdict, level = "외부에서 LOW 로 붙잡혀 있음 (GND 에 닿았거나 센서가 끌어내림)", "err"
            elif up == "hi" and down == "hi":
                verdict = ("정상: 보드의 풀업 저항이 HIGH 로 유지" if p in (2, 3)
                           else "외부에서 HIGH (센서 보드 풀업. 단 3V3 이 없어도 SDA/SCL 로 역전원돼 HIGH 일 수 있어 전원 증거로는 약함)")
                level = "ok"
            elif up == "hi" and down == "lo":
                verdict, level = "아무것도 연결 안 된 것처럼 떠 있음 (선 단선 또는 센서 전원 없음 의심)", "warn"
            else:
                verdict, level = "판단 불가 (풀업=LOW, 풀다운=HIGH)", "warn"
            msgs.append({"pin": p, "name": name, "phys": phys, "pullup": up, "pulldown": down,
                         "verdict": verdict, "level": level})
            self.add_log(f"풀 테스트 {name}(핀{phys}): 풀업→{up} 풀다운→{down} ⇒ {verdict}", level)
        return {"ok": True, "results": msgs}

    def act_recover(self):
        """I2C 버스 복구: SCL 을 9번 토글한 뒤 STOP 조건. 센서가 SDA 를 붙잡은 경우 풀릴 수 있음."""
        before = self.hw.pins()
        self.hw.pinctrl(2, "ip", "pu")
        self.hw.pinctrl(3, "ip", "pu")
        scl_stuck = self.hw.pins().get(3, {}).get("level") == "lo"
        for _ in range(9):
            self.hw.pinctrl(3, "op", "dl")
            time.sleep(0.0001)
            self.hw.pinctrl(3, "ip", "pu")
            time.sleep(0.0001)
        # STOP: SCL HIGH 인 동안 SDA LOW → HIGH
        self.hw.pinctrl(2, "op", "dl")
        self.hw.pinctrl(2, "ip", "pu")
        after = self.hw.pins()
        self._restore(before)
        sda, scl = after.get(2, {}).get("level"), after.get(3, {}).get("level")
        if sda == "hi" and scl == "hi":
            self.add_log("버스 복구 성공: SDA/SCL 이 HIGH 로 돌아왔습니다", "ok")
        else:
            why = " SCL 자체가 LOW 로 고정 → 토글이 안 먹음, 센서 리셋/전원 차단 필요" if scl_stuck else ""
            self.add_log(f"버스 복구 후에도 SDA={sda} SCL={scl}.{why}", "warn")
        return {"ok": True, "sda": sda, "scl": scl}

    def act_reset(self):
        """BNO086 RST 를 50ms 동안 LOW 로 내렸다 놓는다."""
        p = self.reset_bcm
        self.hw.pinctrl(p, "op", "dl")
        time.sleep(0.05)
        self.hw.pinctrl(p, "ip", "pn")
        time.sleep(0.3)
        pins = self.hw.pins()
        sda, scl, rst = (pins.get(x, {}).get("level") for x in (2, 3, p))
        self.add_log(f"RST(GPIO{p}) 리셋 펄스 보냄 → SDA={sda} SCL={scl} RST={rst}",
                     "ok" if sda == scl == "hi" else "warn")
        return {"ok": True, "sda": sda, "scl": scl, "rst": rst}

    @property
    def reset_bcm(self):
        return self.hw.reset_bcm

    def act_restore(self):
        self._restore(None)
        self.add_log("핀 기능을 기본값으로 되돌렸습니다 (SDA/SCL=I2C, RST/INT=입력)", "info")
        return {"ok": True}

    def _restore(self, before):
        self.hw.pinctrl("2,3", "a0", "pn")
        for p in (24, 17):
            st = (before or {}).get(p)
            self.hw.pinctrl(p, st["func"] if st and st["func"] in ("ip", "op") else "ip",
                            st["pull"] if st and st["pull"] in ("pu", "pd", "pn") else "pn")


# ---------------------------------------------------------------- HTTP
def make_handler(mon):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8")
            elif path == "/api/state":
                self._send(200, json.dumps(mon.snapshot).encode(), "application/json")
            elif path == "/api/stream":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                seen = -1
                try:
                    while True:
                        with mon.cv:
                            mon.cv.wait_for(lambda: mon.seq != seen, timeout=5)
                            seen = mon.seq
                        self.wfile.write(b"data: " + json.dumps(mon.snapshot).encode() + b"\n\n")
                        self.wfile.flush()
                        time.sleep(0.2)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            path = urlparse(self.path).path
            if not path.startswith("/api/action/"):
                return self._send(404, b"not found", "text/plain")
            res = mon.action(path.rsplit("/", 1)[-1])
            self._send(200, json.dumps(res).encode(), "application/json")

    return Handler


def main():
    ap = argparse.ArgumentParser(description="eye-pi 하드웨어 진단 화면")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8081)
    ap.add_argument("--bus", type=int, default=1)
    ap.add_argument("--reset-pin", type=int, default=24, metavar="BCM", help="BNO086 RST 의 GPIO (기본 24)")
    ap.add_argument("--shunt", type=float, default=0.1, help="INA219 션트 저항 Ω (기본 0.1)")
    ap.add_argument("--mock", action="store_true", help="가짜 하드웨어로 화면만 확인")
    args = ap.parse_args()

    hw = (MockHw if args.mock else RealHw)(args.bus, args.reset_pin)
    mon = Monitor(hw, args.shunt)
    threading.Thread(target=mon.loop, daemon=True).start()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(mon))
    print(f"하드웨어 진단: http://eye-pi-1.local:{args.port}  (이 컴퓨터에서는 http://localhost:{args.port})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
