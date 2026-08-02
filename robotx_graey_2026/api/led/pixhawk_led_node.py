#!/usr/bin/env python3
"""Decide the status colour and publish /graey/led_state.

RED    (1) = disarmed / emergency motor off
YELLOW (2) = armed and a HUMAN is driving (MANUAL/STABILIZE/ACRO/ALT_HOLD)
GREEN  (3) = armed and CODE is driving - either ArduSub is in an autonomous
             mode, or a node latched /graey/autonomy_active while RC-overriding
             in a pilot mode (the Crusader dp_hold case).

Autonomy-by-override cannot be inferred from flight mode, so the driving node
must assert it and keep asserting it; green lapses after AUTONOMY_TIMEOUT.
"""
from rclpy.node import Node
from std_msgs.msg import Int32, Bool

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.pixhawk.mavlink import Link, mavutil

AUTONOMY_TIMEOUT = 2.0
AUTO_MODES = {'AUTO', 'GUIDED', 'CIRCLE', 'POSHOLD', 'SURFACE', 'RTL'}


class PixhawkLedNode(Node):
    def __init__(self):
        super().__init__('pixhawk_led_node')
        self.declare_parameter('mavlink', 'udpout:127.0.0.1:14552')
        self.link = Link(self.get_parameter('mavlink').value, 196, self.get_logger())

        self.armed = False
        self.mode = '?'
        self.auto_until = 0.0
        self.last_state = None
        self.pub = self.create_publisher(Int32, '/graey/led_state', 10)
        self.create_subscription(Bool, '/graey/autonomy_active', self.on_auto, 10)
        self.create_timer(1.0, self.link.heartbeat)   # or MAVProxy forgets us on restart
        self.create_timer(0.05, self.pump)
        self.create_timer(0.2, self.publish_state)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_auto(self, msg):
        self.auto_until = self.now() + AUTONOMY_TIMEOUT if msg.data else 0.0

    def pump(self):
        def handle(kind, m):
            if kind == 'HEARTBEAT' and m.get_srcComponent() == 1:
                self.armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = self.link.mav.flightmode
        self.link.drain(handle)

    def publish_state(self):
        if not self.armed:
            state = 1
        elif self.mode in AUTO_MODES or self.now() < self.auto_until:
            state = 3
        else:
            state = 2
        if state != self.last_state:
            self.get_logger().info(f'state={state} armed={self.armed} mode={self.mode}')
            self.last_state = state
        self.pub.publish(Int32(data=state))


def main():
    run(PixhawkLedNode)
