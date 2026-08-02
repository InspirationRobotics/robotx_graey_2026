#!/usr/bin/env python3
"""Status LED driver: /graey/led_state (Int32) -> Arduino serial.

0=off 1=RED e-stop 2=YELLOW manual 3=GREEN autonomous. Writes at 5 Hz, which
also feeds the sketch's 3 s watchdog. Auto-reconnects.

Falls back to RED if no state message arrives for STALE_S. Without that, a dead
pixhawk_led_node leaves us writing the last colour forever - the sketch's
watchdog never fires because we are still talking to it, and a stale GREEN can
sit on the prequal video.
"""
import serial
from rclpy.node import Node
from std_msgs.msg import Int32

from robotx_graey_2026.api.node_util import run

DEFAULT_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
STALE_S = 1.0                                       # no state message this long -> RED


class LedNode(Node):
    def __init__(self):
        super().__init__('led_node')
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baud', 115200)
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.state = 1                              # RED until told otherwise
        self.last_msg = 0.0
        self.ser = None
        self.create_subscription(Int32, '/graey/led_state', self.on_state, 10)
        self.create_timer(0.2, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        if 0 <= msg.data <= 3:
            self.state = msg.data
            self.last_msg = self.now()

    def tick(self):
        if self.ser is None:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
                self.get_logger().info(f'LED serial open on {self.port}')
            except OSError as e:
                self.get_logger().warn(f'LED serial open failed: {e}')
            return
        state = self.state if self.now() - self.last_msg < STALE_S else 1
        try:
            self.ser.reset_input_buffer()           # sketch echoes; nobody reads it
            self.ser.write(str(state).encode())
        except OSError as e:
            self.get_logger().warn(f'LED write failed, reconnecting: {e}')
            self.ser = None


def main():
    run(LedNode)
