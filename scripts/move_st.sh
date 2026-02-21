#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WIN_SCRIPT="$(wslpath -w "$ROOT_DIR/scripts/st_id1_com13_move.py")"

powershell.exe -NoProfile -Command "& py -3 '$WIN_SCRIPT' $*"
