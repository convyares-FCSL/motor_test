#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -f "$ROOT_DIR/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$ROOT_DIR/.env"
  set +a
fi

BASE_URL="${STS_FORWARD_BASE_URL:-http://192.168.4.1}"
TIMEOUT_S="${STS_FORWARD_HTTP_TIMEOUT_S:-2}"

usage() {
  echo "Usage: $0 <on|off> [--base-url URL] [--timeout SEC]"
  echo "Example: $0 on --base-url http://192.168.4.1"
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

MODE="$1"
shift || true

while [[ $# -gt 0 ]]; do
  case "$1" in
    --base-url)
      BASE_URL="$2"
      shift 2
      ;;
    --timeout)
      TIMEOUT_S="$2"
      shift 2
      ;;
    *)
      echo "Unknown arg: $1"
      usage
      exit 1
      ;;
  esac
done

if [[ "$MODE" == "on" ]]; then
  CMD_I=14
elif [[ "$MODE" == "off" ]]; then
  CMD_I=15
else
  echo "Invalid mode: $MODE"
  usage
  exit 1
fi

if ! command -v curl >/dev/null 2>&1; then
  echo "Missing dependency: curl"
  echo "Install: sudo apt-get install curl"
  exit 1
fi

URL="${BASE_URL%/}/cmd?inputT=1&inputI=${CMD_I}&inputA=0&inputB=0"
echo "[STEP] Sending serial forwarding '$MODE' to: $URL"

# Firmware endpoint returns plain text; any HTTP 2xx is success for command dispatch.
HTTP_CODE="$(curl -sS -o /tmp/sts_forwarding_resp.txt -m "$TIMEOUT_S" -w "%{http_code}" "$URL")"
RESP_BODY="$(cat /tmp/sts_forwarding_resp.txt || true)"
rm -f /tmp/sts_forwarding_resp.txt

if [[ "$HTTP_CODE" =~ ^2 ]]; then
  echo "[OK] Forwarding command accepted (HTTP $HTTP_CODE)"
  if [[ -n "$RESP_BODY" ]]; then
    echo "[INFO] Response: $RESP_BODY"
  fi
  exit 0
fi

echo "[ERROR] Forwarding command failed (HTTP $HTTP_CODE)"
if [[ -n "$RESP_BODY" ]]; then
  echo "[INFO] Response: $RESP_BODY"
fi
exit 2
