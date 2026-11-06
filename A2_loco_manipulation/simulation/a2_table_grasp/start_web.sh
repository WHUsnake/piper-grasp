#!/usr/bin/env bash
set -euo pipefail
SIM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PORT="${1:-8766}"
printf 'A2 × PiPER 抓取演示：http://127.0.0.1:%s/\n' "$PORT"
exec python3 "$SIM_ROOT/server.py" --port "$PORT"
