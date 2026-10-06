#!/usr/bin/env bash
# Pi 상태 한 번에 점검 (sudo 필요 없음). Pi 에서: bash tools/pi_status.sh
cd "$(dirname "$0")/.." || exit 1
line() { echo; echo "=== $* ==="; }

line "git"
git status -sb | head -5
git log --oneline -3

line "I2C 설정 (/boot/firmware/config.txt)"
grep -nE "i2c|baudrate|googlevoicehat" /boot/firmware/config.txt
echo "현재 실제 I2C 속도: $(od -An -tu4 --endian=big /sys/class/i2c-adapter/i2c-1/of_node/clock-frequency 2>/dev/null | tr -d ' ' || echo '알 수 없음') Hz"
ls -l /dev/i2c-1 2>&1

line "I2C 스캔 (0x4b 또는 0x4a 가 보여야 함)"
i2cdetect -y 1 2>&1

line "전원 / 가동 시간"
uptime
vcgencmd get_throttled 2>&1   # throttled=0x0 이면 정상
dmesg 2>/dev/null | grep -iE "i2c|under-voltage" | tail -5

line "센서를 쓰고 있는 프로세스"
pgrep -af "server.py|test_imu.py|imu_diag.py" || echo "(없음)"
systemctl is-active eye-pi-dashboard 2>/dev/null || true
