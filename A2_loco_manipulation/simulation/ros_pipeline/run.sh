#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -n "${PIPER_MSGS_SETUP:-}" && -f "$PIPER_MSGS_SETUP" ]]; then
  source "$PIPER_MSGS_SETUP"
fi
source "$ROOT/workspaces/pipeline/install/setup.bash"
export ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=1 PYTHONUNBUFFERED=1
exec /usr/bin/python3 "$ROOT/simulation/ros_pipeline/run.py" "$@"
