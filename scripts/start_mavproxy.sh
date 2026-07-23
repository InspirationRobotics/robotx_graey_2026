#!/bin/bash
# MAVProxy = ONLY owner of the Pixhawk USB port.
# usage: ./start_mavproxy.sh <LAPTOP_IP>   (tether = 192.168.2.1)
# by-id path is stable; /dev/ttyACM* numbering changes when the Cube reboots.
CUBE=/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_24002A000B51303231383439-if00

if [ -z "$1" ]; then
  echo "############################################################"
  echo "WARNING: no laptop IP given - defaulting to 192.168.8.137"
  echo "         (that is Chris's laptop on WiFi ONLY)"
  echo "         If QGroundControl never connects, that is why."
  echo "         Pass YOUR laptop IP:  ./start_mavproxy.sh <IP>"
  echo "         Tether = 192.168.2.1"
  echo "############################################################"
fi
LAPTOP=${1:-192.168.8.137}

if [ ! -e "$CUBE" ]; then
  echo "Cube not found at $CUBE - is it powered / finished rebooting?"
  ls -l /dev/serial/by-id/ 2>/dev/null
  exit 1
fi

mavproxy.py --master=$CUBE --baudrate=115200 \
  --out=udpout:$LAPTOP:14550 \
  --out=udpin:0.0.0.0:14551 \
  --out=udpin:0.0.0.0:14552
