#!/usr/bin/env python3
"""Shared entry point. Every node's main() was these same twelve lines."""
import os
import sys

import rclpy


def run(node_cls):
    rclpy.init()
    node = node_cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()                 # nodes owning hardware release it here
        if rclpy.ok():
            rclpy.shutdown()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)                         # skip interpreter teardown: force-killing a
                                            # thread parked in torch's C++ ends in
                                            # std::terminate and a 30 s hang on Ctrl-C
