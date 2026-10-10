"""Offline ROS parameter contract: no serial port or MAVLink connection."""
import importlib.util
import unittest
from unittest.mock import patch


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'Requires ROS environment')
class FrameParameters(unittest.TestCase):
    def check_offset(self, offset):
        import rclpy
        from rclpy.parameter import Parameter
        from robotx_graey_2026.api.navigation import vn100_node, nav_ekf_bridge
        rclpy.init(args=['--ros-args', '-p', 'yaw_offset_deg:='+str(offset)])
        nodes = []
        try:
            with patch.object(vn100_node.serial, 'Serial') as serial, \
                    patch.object(nav_ekf_bridge, 'Link'):
                nodes.append(vn100_node.VN100Node())
                nodes.append(nav_ekf_bridge.NavEKFBridge())
                for node in nodes:
                    self.assertEqual(node.yaw_off, offset)
                    result = node.set_parameters([Parameter('yaw_offset_deg', value=offset+5.0)])
                    self.assertFalse(result[0].successful)
                    self.assertEqual(node.get_parameter('yaw_offset_deg').value, offset)
                    self.assertEqual(node.yaw_off, offset)
                result = nodes[0].set_parameters([Parameter('flip_180', value=False)])
                self.assertFalse(result[0].successful)
                self.assertTrue(nodes[0].flip)
                serial.assert_not_called()
        finally:
            for node in nodes:
                node.destroy_node()
            rclpy.shutdown()

    def test_zero_offset_cannot_silently_change(self):
        self.check_offset(0.0)

    def test_startup_override_still_applies(self):
        self.check_offset(180.0)
