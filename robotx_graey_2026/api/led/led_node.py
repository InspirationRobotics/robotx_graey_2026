#!/usr/bin/env python3
"""Graey status LED strip driver: /graey/led_state (Int32) -> Arduino serial.
0=off 1=RED e-stop 2=YELLOW manual 3=GREEN autonomous. Writes at 5 Hz, which
also feeds the sketch's 3 s watchdog. Auto-reconnects."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

DEFAULT_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'


class LedNode(Node):
    def __init__(self):
        super().__init__('led_node')
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baud', 115200)
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.state = 1
        self.ser = None
        self.create_subscription(Int32, '/graey/led_state', self.on_state, 10)
        self.create_timer(0.2, self.tick)

    def open_port(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
            self.get_logger().info(f'LED serial open on {self.port}')
        except OSError as e:
            self.get_logger().warn(f'LED serial open failed: {e}')
            self.ser = None

    def on_state(self, msg):
        if 0 <= msg.data <= 3:
            self.state = msg.data

    def tick(self):
        if self.ser is None:
            self.open_port()
            return
        try:
            self.ser.write(str(self.state).encode())
        except OSError as e:
            self.get_logger().warn(f'LED write failed, reconnecting: {e}')
            self.ser = None


def main():
    rclpy.init()
    node = LedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
