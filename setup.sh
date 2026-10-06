#!/usr/bin/env bash
# Raspberry Pi Zero용 초기 설정: I2C(BNO086) + I2S(MEMS 마이크) + 파이썬 환경
# 사용법:  bash setup.sh   (일반 사용자로 실행, 내부에서 sudo 사용)
set -euo pipefail

if [ "$(id -u)" -eq 0 ]; then
  echo "root가 아닌 일반 사용자로 실행하세요. (필요한 곳에서 sudo를 자동으로 씁니다)"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Bookworm 이후는 /boot/firmware/config.txt, 이전(Bullseye)은 /boot/config.txt
if [ -f /boot/firmware/config.txt ]; then
  CONFIG=/boot/firmware/config.txt
elif [ -f /boot/config.txt ]; then
  CONFIG=/boot/config.txt
else
  echo "config.txt를 찾을 수 없습니다. Raspberry Pi OS에서 실행 중인지 확인하세요."
  exit 1
fi

echo "==> [1/4] 필요한 패키지 설치"
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip python3-numpy i2c-tools alsa-utils git

echo "==> [2/4] $CONFIG 설정 (I2C + I2S)"
sudo cp -n "$CONFIG" "$CONFIG.bak-eye-pi" || true

# 마지막 섹션이 [all]이 아니면 [all]을 먼저 추가 (다른 보드 섹션에 설정이 들어가는 것 방지)
last_section="$(grep -E '^\[.*\]$' "$CONFIG" | tail -n 1 || true)"
if [ -n "$last_section" ] && [ "$last_section" != "[all]" ]; then
  echo "[all]" | sudo tee -a "$CONFIG" >/dev/null
fi

add_line() {
  local line="$1"
  if grep -qxF "$line" "$CONFIG"; then
    echo "  이미 있음: $line"
  else
    echo "$line" | sudo tee -a "$CONFIG" >/dev/null
    echo "  추가: $line"
  fi
}

# I2C: BNO086 (SDA=GPIO2/핀3, SCL=GPIO3/핀5)
add_line "dtparam=i2c_arm=on"
# BNO08x는 I2C clock stretching을 써서 속도를 400kHz로 올리는 것이 권장됨
add_line "dtparam=i2c_arm_baudrate=400000"
# I2S: MEMS 마이크 (BCLK=GPIO18/핀12, LRCL=GPIO19/핀35, DOUT=GPIO20/핀38)
# Adafruit 가이드: googlevoicehat-soundcard 오버레이 (48kHz 고정, 볼륨 컨트롤 없음)
add_line "dtoverlay=googlevoicehat-soundcard"

# i2c-dev 모듈 자동 로드
if ! grep -qxF "i2c-dev" /etc/modules; then
  echo "i2c-dev" | sudo tee -a /etc/modules >/dev/null
fi

echo "==> [3/4] 사용자 권한 (i2c, audio, gpio 그룹)"
for g in i2c audio gpio; do
  if getent group "$g" >/dev/null; then
    sudo usermod -aG "$g" "$USER" || true
  fi
done

echo "==> [4/4] 파이썬 가상환경 + 라이브러리"
cd "$SCRIPT_DIR"
python3 -m venv --system-site-packages .venv
# shellcheck disable=SC1091
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

echo
echo "설정 완료. I2C/I2S 오버레이를 적용하려면 재부팅이 필요합니다:"
echo "  sudo reboot"
echo
echo "재부팅 후 확인:"
echo "  ls /dev/i2c-1            # 있어야 함"
echo "  arecord -l               # googlevoicehat 카드가 보여야 함"
echo "  source .venv/bin/activate && python check_all.py"
