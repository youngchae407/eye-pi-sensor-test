#!/usr/bin/env bash
# 대시보드를 Pi가 켜질 때마다 자동으로 실행되게 등록합니다.
# 사용법:  bash install_service.sh        (제거: bash install_service.sh --remove)
set -euo pipefail

NAME=eye-pi-dashboard
UNIT="/etc/systemd/system/$NAME.service"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="$DIR/.venv/bin/python"

if [ "${1:-}" = "--remove" ]; then
  sudo systemctl disable --now "$NAME" 2>/dev/null || true
  sudo rm -f "$UNIT"
  sudo systemctl daemon-reload
  echo "자동 실행을 해제했습니다."
  exit 0
fi

if [ ! -x "$PY" ]; then
  echo ".venv가 없습니다. 먼저 bash setup.sh 를 실행하고 재부팅하세요."
  exit 1
fi

sudo tee "$UNIT" >/dev/null <<EOF
[Unit]
Description=eye-pi sensor dashboard
After=network-online.target sound.target
Wants=network-online.target

[Service]
User=$USER
WorkingDirectory=$DIR
ExecStart=$PY $DIR/server.py --port 8000
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now "$NAME"

echo
echo "등록 완료. 이제 Pi가 켜지면 대시보드가 자동으로 시작됩니다."
echo "  Mac 브라우저:  http://$(hostname).local:8000"
echo "  상태 보기:     systemctl status $NAME"
echo "  로그 보기:     journalctl -u $NAME -f"
echo "  잠시 멈추기:   sudo systemctl stop $NAME   (터미널에서 test_mic.py를 직접 돌릴 때 필요)"
