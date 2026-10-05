#!/usr/bin/env python3
"""Passes the VN-100 and DVL readings out of the ROS container to sonar_map.py.

The sonar tools run on the Jetson itself, outside the `graey` container, where
there is no ROS. This runs inside it, like gui_node does, and forwards every
reading the moment it arrives as one small UDP packet. Nothing else uses it.

sonar_map.py starts it, and it quits by itself when sonar_map.py ends, because
its input closes then. To run it by hand instead (sonar_map.py --no-relay):

    docker exec -it graey bash -c "source /opt/ros/humble/setup.bash && \
        python3 /root/robotx_ws/src/robotx_graey_2026/tools/pose_relay.py"
"""
import json
import os
import socket
import sys
import threading

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

PORT = 14660            # must match RELAY_PORT in api/sonar/pose.py


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else PORT
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(d):
        sock.sendto(json.dumps(d).encode(), ("127.0.0.1", port))

    def att(m):
        o = m.orientation
        send({"att": [o.w, o.x, o.y, o.z]})

    def vel(m):
        v = m.twist.twist.linear
        send({"vel": [v.x, v.y, v.z]})

    # quit when whoever started us goes away (by hand: Ctrl-C or Ctrl-D)
    threading.Thread(target=lambda: (sys.stdin.read(), os._exit(0)), daemon=True).start()

    rclpy.init()
    node = Node("pose_relay")
    node.create_subscription(Imu, "/graey/vn100/imu", att, 20)
    node.create_subscription(TwistWithCovarianceStamped, "/graey/dvl/velocity", vel, 10)
    node.create_subscription(Bool, "/graey/dvl/valid", lambda m: send({"ok": m.data}), 10)
    print(f"relaying VN-100 + DVL to udp 127.0.0.1:{port}  (Ctrl-C to stop)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
