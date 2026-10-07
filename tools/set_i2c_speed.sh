#!/usr/bin/env bash
# I2C 속도 변경 (sudo 비밀번호 필요, 적용하려면 재부팅).
# 사용 예 (Pi 에서): sudo bash tools/set_i2c_speed.sh 10000
# BNO08x 권장(Adafruit 클럭 스트레칭 가이드): 10000, 안 되면 5000 → 1000
set -e
SPEED="${1:?속도(Hz)를 주세요. 예: 10000, 5000, 1000}"
CFG=/boot/firmware/config.txt
[ "$(id -u)" -eq 0 ] || { echo "sudo 로 실행하세요: sudo bash $0 $SPEED"; exit 1; }
cp "$CFG" "$CFG.bak.$(date +%Y%m%d%H%M%S)"
if grep -q "^dtparam=i2c_arm_baudrate=" "$CFG"; then
  sed -i "s/^dtparam=i2c_arm_baudrate=.*/dtparam=i2c_arm_baudrate=$SPEED/" "$CFG"
else
  echo "dtparam=i2c_arm_baudrate=$SPEED" >> "$CFG"
fi
grep -n "i2c_arm" "$CFG"
echo "변경 완료. 재부팅해야 적용됩니다: sudo reboot"
