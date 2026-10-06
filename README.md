# eye-pi sensor test

Raspberry Pi Zero(`eye-pi-1`)에 연결한 **I2S MEMS 마이크**와 **SparkFun BNO086 IMU**가 제대로 동작하는지 확인하는 도구입니다.

- **터미널 테스트**: `test_mic.py`, `test_imu.py`, `check_all.py`
- **웹 대시보드**: `server.py` — Mac 브라우저에서 3D 자세, 마이크 음량, 연결 상태, 터미널 점검 결과를 실시간으로 봅니다.

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
> **배선은 Pi 전원을 끈 상태에서** 하세요.

## 2. 설정 (Pi에서 한 번만)

```bash
git clone https://github.com/youngchae407/eye-pi-sensor-test.git
cd eye-pi-sensor-test
bash setup.sh
sudo reboot
```

`setup.sh`가 하는 일:

- `config.txt`에 추가 (`/boot/firmware/config.txt`, 구버전은 `/boot/config.txt`; 원본은 `.bak-eye-pi`로 백업)
  - `dtparam=i2c_arm=on` — I2C 켜기
  - `dtparam=i2c_arm_baudrate=...` — I2C 속도 (BNO08x는 400000 권장이지만 Pi Zero에서는 100000 이하로 낮춰 시험 중)
  - `dtoverlay=googlevoicehat-soundcard` — I2S 마이크 드라이버 (Adafruit 가이드 방식)
- `i2c-dev` 모듈 로드, 사용자를 `i2c`/`audio`/`gpio` 그룹에 추가
- 필요한 apt 패키지, `.venv` 가상환경, 파이썬 라이브러리 설치

## 3. 웹 대시보드

```bash
cd eye-pi-sensor-test
source .venv/bin/activate
python server.py
```

Mac 브라우저에서 **http://eye-pi-1.local:8080** 을 엽니다. (`.local`이 안 열리면 서버를 켤 때 출력되는 IP 주소로 접속하세요.)

화면 구성:

- **3D 보드**: IMU 자세(회전벡터)가 실시간으로 반영됩니다. 드래그하면 시점이 돌아가고, `정면으로 맞추기`로 지금 방향을 Yaw 0°로 정합니다.
- **마이크 음량**: 현재 크기(dBFS), 최근 최고점, 최근 30초 그래프. 0이 최대, 조용한 방은 −60 아래입니다.
- **센서 원시값**: 가속도 / 자이로 / 지자기 (X, Y, Z)
- **연결 상태**: IMU·마이크 연결, I2C 장치 주소, 방향 정확도(지자기 보정), Pi 부하와 메모리
- **터미널**: `i2cdetect`, `arecord -l`, 마이크/IMU 테스트, 전체 점검을 버튼으로 실행하고 결과를 그대로 봅니다. 서버 로그도 여기에 나옵니다.

옵션:

```bash
python server.py --port 9000         # 포트 변경 (기본 8080)
python server.py --reset-pin 24      # BNO086 RST를 GPIO24에 연결했을 때 하드웨어 리셋 사용
python server.py --game-rotation     # 지자기 없이 방향 계산 (실내에서 Yaw가 흔들릴 때)
python server.py --no-mic            # 마이크 읽기 끄기 (--no-imu 도 있음)
```

**센서 없이 화면만 보기**: Mac에서도 `python3 server.py --mock` 을 실행하면 가짜 데이터로 대시보드가 움직입니다 (`http://localhost:8080`).

**부팅할 때 자동 시작**:

```bash
bash install_service.sh              # 등록 (해제: bash install_service.sh --remove)
```

주의:

- 마이크와 IMU는 **한 프로세스만** 쓸 수 있습니다. 대시보드가 켜져 있을 때 터미널에서 `python test_mic.py`를 직접 돌리면 `device busy`가 납니다. 대시보드의 **터미널 버튼**을 쓰거나(자동으로 잠시 멈췄다 다시 연결), `sudo systemctl stop eye-pi-dashboard`로 멈추세요.
- 대시보드에는 로그인이 없습니다. **집/사무실 같은 믿을 수 있는 네트워크에서만** 쓰고, 공용 네트워크에서는 켜 두지 마세요.
- 방향(Yaw)은 지자기를 쓰기 때문에 모니터·스피커·금속 근처에서 틀어질 수 있습니다. 화면의 `방향 정확도`가 `높음`이 될 때까지 센서를 8자로 천천히 돌려 주세요.

## 4. 터미널 테스트

```bash
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

## 5. 문제 해결

**마이크**
- `arecord -l`에 카드가 안 보임 → `config.txt`에 `dtoverlay=googlevoicehat-soundcard`가 있는지, 재부팅했는지 확인
- 카드가 보이는데 값이 전부 0 / 고정 → DOUT·BCLK·LRCL 배선, SEL이 GND(또는 3.3V)에 연결됐는지 확인 (카드는 드라이버가 만드는 거라 배선이 없어도 보입니다)
- 볼륨 조절은 이 드라이버에서 지원되지 않습니다 (Adafruit 가이드의 `.asoundrc` softvol 방식 참고)

**IMU**
- `i2cdetect -y 1`에서 `4b`(또는 `4a`)가 안 보임 → 전원/SDA/SCL 배선, 납땜 상태 확인
- 초기화 오류가 가끔 남 → 센서 전원을 껐다 켜고 재시도 (BNO08x의 알려진 특성)
- `Unprocessable Batch bytes` 오류 → 센서가 명령 채널(0)로 보낸 짧은 메시지(예: `01 0E`)를 라이브러리(v1.3.3)가
  센서 데이터로 잘못 읽어서 나는 오류입니다. `imu_init.py`가 이 패킷을 건너뛰도록 라이브러리를 패치합니다
  (`server.py`, `test_imu.py`에 자동 적용). 그래도 실패하면:
  1. 진단: 서버를 멈추고 `python tools/imu_diag.py` (RST 연결 시 `--reset-pin 24`) — 어떤 패킷이 오가는지 출력
  2. 센서 3V3를 뽑았다 꽂아 완전히 전원 리셋
  3. BNO086 **RST**를 Pi GPIO24(핀 18)에 연결하고 `python server.py --reset-pin 24` (하드웨어 리셋)
  4. I2C 속도 바꾸기: `sudo bash tools/set_i2c_speed.sh 50000` 후 `sudo reboot` (100000 → 50000 → 10000 순서로)
  5. SDA/SCL 선을 짧게, 점퍼 접촉 확인
- Pi 상태를 한 번에 보기: `bash tools/pi_status.sh` (git, I2C 설정·스캔, 전원, 실행 중인 서버)
- 가속도 크기가 9.8에서 크게 벗어남 → 테스트 중 보드를 움직이고 있지 않은지 확인

**대시보드**
- 브라우저에서 `eye-pi-1.local:8080`이 안 열림 → `ssh`로 접속해 `python server.py`가 실행 중인지(또는 `systemctl status eye-pi-dashboard`) 확인
- 3D 보드가 흐리게 보임 → IMU 연결 상태를 알려 주는 안내 문구가 화면 가운데에 나옵니다

## 6. 개발 흐름 (GitHub)

PC에서 수정 → GitHub에 push → Pi에서 pull:

```bash
# PC
git add -A && git commit -m "메시지" && git push

# Pi (ssh admin@eye-pi-1.local 로 접속 후)
cd eye-pi-sensor-test && git pull
sudo systemctl restart eye-pi-dashboard    # 자동 시작을 등록했다면
```
