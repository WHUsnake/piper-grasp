#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/../runtime/piper_msgs/share/piper_msgs/local_setup.bash"
export PATH="/usr/bin:/bin:/opt/ros/humble/bin:$PATH"
exec colcon --log-base "$ROOT/workspaces/pipeline/log" build \
 --base-paths "$ROOT/src/interfaces/ros2/grasp_msgs" "$ROOT/src/manipulation" "$ROOT/src/perception" \
 --build-base "$ROOT/workspaces/pipeline/build" --install-base "$ROOT/workspaces/pipeline/install" \
 --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
