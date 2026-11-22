#!/usr/bin/env bash
set -euo pipefail
: "${LAB_REPO:?Set LAB_REPO to the full robot-lab checkout with this release overlay installed}"
if [[ ! -x "$LAB_REPO/bin/lab" ]]; then
  echo "Missing Robot Lab bin/lab at $LAB_REPO" >&2
  exit 2
fi
export LAB_GRASP_BACKEND=graspnet
exec "$LAB_REPO/bin/lab" --profile hssd-go2-piper preview --development
