#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -f "$ROOT_DIR/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$ROOT_DIR/.env"
  set +a
fi

if ! python3 -c "import aiohttp" >/dev/null 2>&1; then
  echo "Missing dependency: aiohttp"
  echo "Install with one of:"
  echo "  sudo apt-get install python3-aiohttp"
  echo "  python3 -m venv .venv && source .venv/bin/activate && pip install aiohttp"
  exit 1
fi

if [[ "${STS_AUTO_FORWARD_ON:-0}" =~ ^(1|true|yes|on)$ ]]; then
  if [[ -x "$ROOT_DIR/scripts/serial_forwarding.sh" ]]; then
    "$ROOT_DIR/scripts/serial_forwarding.sh" on
    if [[ -n "${STS_FORWARD_SETTLE_S:-}" ]]; then
      sleep "$STS_FORWARD_SETTLE_S"
    fi
  else
    echo "Warning: STS_AUTO_FORWARD_ON is set but scripts/serial_forwarding.sh is missing"
  fi
fi

if [[ ! -f "$ROOT_DIR/install/setup.bash" ]]; then
  echo "Workspace not built yet. Run: ./scripts/build_ros.sh"
  exit 1
fi

# ROS setup scripts may reference unset tracing vars; source with nounset disabled.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$ROOT_DIR/install/setup.bash"
set -u

exec ros2 launch servo_bringup system.launch.py "$@"
