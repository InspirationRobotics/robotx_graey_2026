#!/usr/bin/env python3
"""Decide the status colour and publish /graey/led_state.

RED    (1) = disarmed / emergency motor off
YELLOW (2) = armed and a HUMAN is driving (MANUAL/STABILIZE/ACRO/ALT_HOLD)
GREEN  (3) = armed and CODE is driving - either ArduSub is in an autonomous
             mode, or a node latched /graey/autonomy_active while RC-overriding
             in a pilot mode (the Crusader dp_hold case).

Autonomy-by-override cannot be inferred from flight mode, so the driving node
must assert it and keep asserting it; green lapses after AUTONOMY_TIMEOUT."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from pymavlink import mavutil

AUTONOMY_TIMEOUT = 2.0
AUTO_MODES = {'AUTO', 'GUIDED', 'CIRCLE', 'POSHOLD', 'SURFACE', 'RTL'}


class PixhawkLedNode(Node):
    def __init__(self):
        super().__init__('pixhawk_led_node')
        self.declare_parameter('mavlink', 'udpout:127.0.0.1:14552')
        conn = self.get_parameter('mavlink').value
        self.get_logger().info(f'opening MAVLink {conn}')
        self.mav = mavutil.mavlink_connection(conn, source_system=1, source_component=196)
        self.mav.mav.heartbeat_send(6, 8, 0, 0, 0)

        self.armed = False
        self.mode = '?'
        self.auto_until = 0.0
        self.last_state = None
        self.pub = self.create_publisher(Int32, '/graey/led_state', 10)
        self.create_subscription(Bool, '/graey/autonomy_active', self.on_auto, 10)
        self.create_timer(0.05, self.pump)
        self.create_timer(0.2, self.publish_state)

    def on_auto(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.auto_until = now + AUTONOMY_TIMEOUT if msg.data else 0.0

    def pump(self):
        while True:
            m = self.mav.recv_match(type='HEARTBEAT', blocking=False)
            if m is None:
                return
            if m.get_srcComponent() == 1:
                self.armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = self.mav.flightmode

    def publish_state(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.armed:
            state = 1
        elif self.mode in AUTO_MODES or now < self.auto_until:
            state = 3
        else:
            state = 2
        if state != self.last_state:
            self.get_logger().info(f'state={state} armed={self.armed} mode={self.mode}')
            self.last_state = state
        self.pub.publish(Int32(data=state))


def main():
    rclpy.init()
    node = PixhawkLedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
