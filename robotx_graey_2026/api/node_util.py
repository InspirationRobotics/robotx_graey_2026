#!/usr/bin/env python3
"""Shared entry point. Every node's main() was these same twelve lines."""
import rclpy


def run(node_cls):
    rclpy.init()
    node = node_cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():                              # a node may shut down on its own
            rclpy.shutdown()
