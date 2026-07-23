#!/bin/bash
# MAVProxy = ONLY owner of the Pixhawk USB port.
# usage: ./start_mavproxy.sh <LAPTOP_IP>
mavproxy.py --master=/dev/ttyACM0 --baudrate=115200 \
  --out=udpout:$1:14550 \
  --out=udpin:0.0.0.0:14551
