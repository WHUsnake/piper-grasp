#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/../runtime/piper_msgs/share/piper_msgs/local_setup.bash"
source "$ROOT/workspaces/pipeline/install/setup.bash"
export ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1 PYTHONUNBUFFERED=1
exec /usr/bin/python3 "$ROOT/simulation/ros_pipeline/run.py" "$@"
