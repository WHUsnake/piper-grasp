#!/usr/bin/env bash
# Build only the message and candidate overlay; never starts ROS nodes/hardware.
set -eo pipefail
GRASP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
export PATH="/usr/bin:/bin:/opt/ros/humble/bin:$PATH"
exec colcon --log-base "$GRASP_ROOT/workspaces/graspnet/log" build \
  --base-paths "$GRASP_ROOT/src/interfaces/ros2/grasp_msgs" "$GRASP_ROOT/src/manipulation/grasp_generation" \
  --build-base "$GRASP_ROOT/workspaces/graspnet/build" \
  --install-base "$GRASP_ROOT/workspaces/graspnet/install" \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
