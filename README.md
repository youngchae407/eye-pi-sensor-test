# eye-pi sensor test

Raspberry Pi Zero(`eye-pi-1`)에 연결한 **I2S MEMS 마이크**와 **SparkFun BNO086 IMU**가 제대로 동작하는지 확인하는 테스트 도구입니다.

| 센서 | 인터페이스 | 비고 |
|---|---|---|
| Adafruit I2S MEMS 마이크 (SPH0645LM4H 또는 ICS-43434) | I2S | `googlevoicehat-soundcard` 오버레이, 48kHz 고정 |
| SparkFun BNO086 IMU | I2C | 주소 `0x4B`(기본) / `0x4A`(점퍼 변경 시) |

## 1. 배선

Pi의 40핀 헤더 기준입니다. 핀 번호는 **물리 핀 번호**, 괄호는 BCM GPIO입니다.

**I2S 마이크**

| 마이크 | Pi 핀 |
|---|---|
| 3V | 핀 17 (3.3V) |
| GND | 핀 14 (GND) |
| SEL | 핀 39 (GND) → 왼쪽 채널 (모노) |
| BCLK | 핀 12 (GPIO18) |
| LRCL | 핀 35 (GPIO19) |
| DOUT | 핀 38 (GPIO20) |

**BNO086 IMU (I2C)**

| BNO086 | Pi 핀 |
|---|---|
| 3V3 | 핀 1 (3.3V) |
| GND | 핀 6 (GND) |
| SDA | 핀 3 (GPIO2) |
| SCL | 핀 5 (GPIO3) |

> 두 센서 모두 **3.3V**에 연결하세요 (5V 금지). I2C와 I2S가 쓰는 핀은 서로 겹치지 않습니다.
> BNO086의 INT/RST 핀은 이 테스트에서는 연결하지 않아도 됩니다.

## 2. 설정 (Pi에서 한 번만)

```bash
git clone https://github.com/<내-아이디>/eye-pi-sensor-test.git
cd eye-pi-sensor-test
bash setup.sh
sudo reboot
```

`setup.sh`가 하는 일:

- `config.txt`에 추가 (`/boot/firmware/config.txt`, 구버전은 `/boot/config.txt`; 원본은 `.bak-eye-pi`로 백업)
  - `dtparam=i2c_arm=on` — I2C 켜기
  - `dtparam=i2c_arm_baudrate=400000` — BNO08x 권장 속도
  - `dtoverlay=googlevoicehat-soundcard` — I2S 마이크 드라이버 (Adafruit 가이드 방식)
- `i2c-dev` 모듈 로드, 사용자를 `i2c`/`audio`/`gpio` 그룹에 추가
- 필요한 apt 패키지, `.venv` 가상환경, 파이썬 라이브러리 설치

## 3. 실행

재부팅 후:

```bash
cd eye-pi-sensor-test
source .venv/bin/activate

python check_all.py            # 마이크 + IMU 한 번에
python test_mic.py             # 마이크만 (박수를 치면 레벨 미터가 움직임)
python test_imu.py             # IMU만 (처음엔 가만히, 그 뒤엔 천천히 돌려 보기)
```

자주 쓰는 옵션:

```bash
python test_mic.py --duration 10 --save test.wav   # 녹음 저장
python test_mic.py --require-sound                 # 소리 변화가 없으면 FAIL
python test_imu.py --address 0x4A --duration 20    # 주소/시간 지정
```

각 스크립트는 PASS면 종료 코드 0, FAIL이면 1을 돌려줍니다.

## 4. 문제 해결

**마이크**
- `arecord -l`에 카드가 안 보임 → `config.txt`에 `dtoverlay=googlevoicehat-soundcard`가 있는지, 재부팅했는지 확인
- 값이 전부 0 / 고정 → DOUT·BCLK·LRCL 배선, SEL이 GND(또는 3.3V)에 연결됐는지 확인
- 볼륨 조절은 이 드라이버에서 지원되지 않습니다 (Adafruit 가이드의 `.asoundrc` softvol 방식 참고)

**IMU**
- `i2cdetect -y 1`에서 `4b`(또는 `4a`)가 안 보임 → 전원/SDA/SCL 배선, 납땜 상태 확인
- 초기화 오류가 가끔 남 → 센서 전원을 껐다 켜고 재시도 (BNO08x의 알려진 특성)
- 가속도 크기가 9.8에서 크게 벗어남 → 테스트 중 보드를 움직이고 있지 않은지 확인

## 5. 개발 흐름 (GitHub)

PC에서 수정 → GitHub에 push → Pi에서 pull:

```bash
# PC
git add -A && git commit -m "메시지" && git push

# Pi (ssh eye-pi-1.local 로 접속 후)
cd eye-pi-sensor-test && git pull
```
