#!/bin/bash
# MAVProxy = ONLY owner of the Pixhawk USB port.
#
#   ./start_mavproxy.sh            broadcast telemetry to every subnet we are on
#   ./start_mavproxy.sh <IP>       send to one address instead (laptop on another subnet)
#
# Broadcasting means any laptop on the router or the tether sees Graey in
# QGroundControl with no configuration. It also means any of them can SEND
# commands - only ONE machine should have joystick input enabled.
#
# by-id path is stable; /dev/ttyACM* numbering changes when the Cube reboots.
CUBE=/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_24002A000B51303231383439-if00

if [ ! -e "$CUBE" ]; then
  echo "Cube not found at $CUBE - is it powered / finished rebooting?"
  ls -l /dev/serial/by-id/ 2>/dev/null
  exit 1
fi

if [ -n "$1" ]; then
  OUTS="--out=udpout:$1:14560"
  echo "telemetry -> $1:14560"
else                                    # one broadcast per live interface, so this
  OUTS=""                               # works on tether only, WiFi only, or both
  for BCAST in $(ip -o -4 addr show scope global | grep -v docker | awk '/brd/ {print $6}'); do
    OUTS="$OUTS --out=udpbcast:$BCAST:14560"
    echo "telemetry -> $BCAST:14560 (broadcast)"
  done
  [ -z "$OUTS" ] && echo "WARNING: no network interfaces up - QGC will not connect"
fi

[ -t 1 ] || DAEMON="--daemon"           # no terminal = running as a service

mavproxy.py --master=$CUBE --baudrate=115200 $DAEMON \
  --state-basedir=/root/robotx_ws/logs \
  $OUTS \
  --out=udpin:0.0.0.0:14551 \
  --out=udpin:0.0.0.0:14552 \
  --out=udpin:0.0.0.0:14553 \
  --out=udpin:0.0.0.0:14554 \
  --out=udpin:0.0.0.0:14555 \
  --out=udpin:0.0.0.0:14556
