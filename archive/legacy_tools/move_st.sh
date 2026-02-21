#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WIN_SCRIPT="$(wslpath -w "$SCRIPT_DIR/st_id1_com13_move.py")"

powershell.exe -NoProfile -Command "& py -3 '$WIN_SCRIPT' $*"
