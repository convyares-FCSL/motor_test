#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

# ROS setup scripts may reference unset tracing vars; source with nounset disabled.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

colcon build --symlink-install "$@"
