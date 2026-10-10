#!/usr/bin/env bash
set -eo pipefail
# Keep simulated ROS sensor/status topics out of the real vehicle's domain.
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1
export GRAEY_RUNTIME_SCOPE=sitl

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="${SITL_RUN_DIR:-$ROOT/sitl/run}"
LOG_DIR="${SITL_LOG_DIR:-$ROOT/sitl/logs}"
SITL_BIN="${SITL_BIN:-/mnt/c/Users/libei/OneDrive/Documents/ChatGPT/RX 2026 GRAEY/output/gps-sitl-2026-10-02/ardusub}"
QGC_HOST="${QGC_HOST:-}"
mkdir -p "$RUN_DIR/scripts" "$LOG_DIR"
if [[ ! -x "$SITL_BIN" ]]; then
  echo "ArduSub SITL binary not found: $SITL_BIN" >&2
  echo "Set SITL_BIN to an ArduSub 4.5.x SITL executable." >&2
  exit 2
fi
if [[ -f "$RUN_DIR/sitl.pid" ]] && kill -0 "$(cat "$RUN_DIR/sitl.pid")" 2>/dev/null; then
  echo "SITL stack already running (PID $(cat "$RUN_DIR/sitl.pid"))." >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
python3 -c 'import rclpy, pymavlink, serial' >/dev/null
command -v mavproxy.py >/dev/null
# Refuse occupied endpoints before launching any simulator or sending commands.
# Separate scopes alone do not isolate network ports.
python3 - <<'PY'
import socket
held = []
try:
    for kind, ports in ((socket.SOCK_STREAM, (5850, 8090)),
                        (socket.SOCK_DGRAM, range(14551, 14560))):
        for port in ports:
            s = socket.socket(socket.AF_INET, kind)
            held.append(s)
            if kind == socket.SOCK_STREAM:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind(('127.0.0.1', port))
            except OSError as error:
                raise RuntimeError(f'SITL endpoint busy: 127.0.0.1:{port} ({kind})') from error
finally:
    for s in held:
        s.close()
PY
cp "$ROOT/sitl/scripts/navigation_source_report.lua" "$RUN_DIR/scripts/"
# Preserve prior evidence, but never mistake yesterday's result for this run.
if [[ -f "$LOG_DIR/scenario-result.json" ]]; then
  mv "$LOG_DIR/scenario-result.json" "$LOG_DIR/scenario-result.previous.$(date +%s%N).json"
fi

cleanup() {
  for file in ros mavproxy sitl; do
    if [[ -f "$RUN_DIR/$file.pid" ]]; then
      kill "$(cat "$RUN_DIR/$file.pid")" 2>/dev/null || true
      rm -f "$RUN_DIR/$file.pid"
    fi
  done
  wait || true
}
trap cleanup EXIT INT TERM

(
  cd "$RUN_DIR"
  "$SITL_BIN" -w --synthetic-clock --model vectored --sysid 1 --home "${HOME_LAT:-32.9240586},${HOME_LON:--117.0385389},0,0" \
    --defaults "$ROOT/params/sitl_graey.parm" --serial0 tcp:5850
) >"$LOG_DIR/ardusub.log" 2>&1 &
echo $! > "$RUN_DIR/sitl.pid"

for _ in $(seq 1 40); do
  if (echo >/dev/tcp/127.0.0.1/5850) >/dev/null 2>&1; then break; fi
  sleep .25
done
if ! (echo >/dev/tcp/127.0.0.1/5850) >/dev/null 2>&1; then
  tail -40 "$LOG_DIR/ardusub.log" >&2
  echo "ArduSub SITL did not open TCP 5850." >&2
  exit 3
fi

MAVPROXY_OUTPUTS=(
  --out=udpin:127.0.0.1:14551 --out=udpin:127.0.0.1:14552 \
  --out=udpin:127.0.0.1:14553 --out=udpin:127.0.0.1:14554 \
  --out=udpin:127.0.0.1:14555 --out=udpin:127.0.0.1:14556 \
  --out=udpin:127.0.0.1:14557 --out=udpin:127.0.0.1:14558 \
  --out=udpin:127.0.0.1:14559
)
# Do not feed the live QGC instance by default: SITL and the physical Cube
# both use MAVLink system ID 1, which makes QGC alternate one vehicle marker.
# Set QGC_HOST explicitly only when using a separate, SITL-only QGC instance.
if [[ -n "$QGC_HOST" ]]; then
  MAVPROXY_OUTPUTS+=(--out=udpout:"$QGC_HOST":14550)
fi
mavproxy.py --master=tcp:127.0.0.1:5850 --non-interactive --default-modules=link,log \
  "${MAVPROXY_OUTPUTS[@]}" --state-basedir="$LOG_DIR" >"$LOG_DIR/mavproxy.log" 2>&1 &
echo $! > "$RUN_DIR/mavproxy.pid"
sleep 2

cd "$ROOT"
colcon build --symlink-install --packages-select robotx_graey_2026
source install/setup.bash
ros2 launch robotx_graey_2026 sitl.launch.py \
  supervisor_mavlink:="${SUPERVISOR_MAVLINK:-udpout:127.0.0.1:14557}" \
  result_file:="$LOG_DIR/scenario-result.json" \
  home_lat:="${HOME_LAT:-32.9240586}" home_lon:="${HOME_LON:--117.0385389}" \
  waypoint_north_m:="${WAYPOINT_NORTH_M:-1.5}" waypoint_east_m:="${WAYPOINT_EAST_M:-1.0}" \
  >"$LOG_DIR/ros.log" 2>&1 &
echo $! > "$RUN_DIR/ros.pid"
echo "SITL running. GUI: http://localhost:8090/navigation"
if [[ -n "$QGC_HOST" ]]; then
  echo "SITL QGC UDP host: ${QGC_HOST}:14550 (use a SITL-only QGC instance)"
else
  echo "SITL QGC output disabled to prevent collision with the physical vehicle."
fi
echo "Logs: $LOG_DIR"
set +e
wait -n
child_status=$?
set -e
echo "A SITL process exited. Recent logs:" >&2
tail -n 80 "$LOG_DIR/ros.log" "$LOG_DIR/mavproxy.log" "$LOG_DIR/ardusub.log" >&2
exit "${child_status:-4}"
