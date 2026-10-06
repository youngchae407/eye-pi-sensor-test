"""BNO086 초기화 도우미 (server.py 와 test_imu.py 가 같이 씁니다).

BNO08x는 Pi의 I2C에서 초기화 도중 가끔 `RuntimeError: ('Unprocessable Batch bytes', N)` 를
내며 실패합니다. 센서가 보낸 짧은 상태 패킷을 라이브러리가 센서 데이터로 잘못 읽는 것으로,
센서를 다시 리셋하고 처음부터 재시도하면 대부분 지나갑니다.
이 모듈은 '객체 생성 + 기능 켜기'를 한 묶음으로 묶어 통째로 재시도하고,
라이브러리가 실패할 때마다 찍는 패킷 덤프도 조용하게 만듭니다.
"""
import importlib
import time


def quiet_library():
    """adafruit_bno08x 가 실패 시 print(packet) 으로 쏟아내는 덤프를 끈다."""
    lib = importlib.import_module("adafruit_bno08x")
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
                return bno, addr
            except Exception as e:  # noqa: BLE001
                last = e
                log(f"IMU 초기화 재시도 {attempt}/{tries} (주소 {hex(addr)}): {type(e).__name__}: {e}", "warn")
                time.sleep(0.5 + 0.3 * attempt)  # 점점 더 길게 쉬었다 다시 리셋
    raise last if last else RuntimeError("BNO086 초기화 실패")
