#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

DO_BUILD=0
if [[ "${1:-}" == "--build" ]]; then
  DO_BUILD=1
  shift
fi

if [[ "$DO_BUILD" -eq 1 ]]; then
  "$ROOT_DIR/scripts/build_ros.sh"
fi

exec "$ROOT_DIR/scripts/launch_ros.sh" "$@"
