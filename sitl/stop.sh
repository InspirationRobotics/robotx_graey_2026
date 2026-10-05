#!/usr/bin/env bash
set -euo pipefail
RUN_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run"
for file in ros mavproxy sitl; do
  if [[ -f "$RUN_DIR/$file.pid" ]]; then
    pid="$(cat "$RUN_DIR/$file.pid")"
    kill "$pid" 2>/dev/null || true
    rm -f "$RUN_DIR/$file.pid"
  fi
done
echo "Stopped local Graey SITL processes."
