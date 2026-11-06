#!/usr/bin/env bash
# Manual commissioning entry, never stops or modifies the existing base service.
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
if systemctl is-active --quiet a2.service; then
  echo 'Refused: existing a2.service is active. Authorized service handover is required.' >&2
  exit 2
fi
if pgrep -f '^/home/unitree/a2_nav/unitree_sdk2/build/bin/a2_move_sub' >/dev/null; then
  echo 'Refused: legacy base process still running.' >&2
  exit 2
fi
exec env -i HOME=/home/unitree PATH=/usr/bin:/bin ROOT="$ROOT" bash --noprofile --norc -c '
  set -e
  source /opt/ros/noetic/setup.bash
  export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_HOSTNAME=localhost
  export LD_LIBRARY_PATH=/home/unitree/a2_nav/unitree_sdk2/thirdparty/lib/x86_64:${LD_LIBRARY_PATH:-}
  exec "$ROOT/workspaces/guarded_base_build/a2_guarded_base" eth0 _allow_task_control:=false
'
