#!/usr/bin/env bash
set -euo pipefail

NOVNC_PORT="${NOVNC_PORT:-8080}"
export NOVNC_PORT

echo "[noVNC] Open: http://localhost:${NOVNC_PORT}/"

exec /usr/bin/supervisord -c /etc/supervisor/conf.d/supervisord.conf
