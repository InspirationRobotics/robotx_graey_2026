"""Graey core stack: status LEDs, DVL/VN-100 navigation into the EKF, and the GUI.

MAVProxy is NOT started here - it owns the Pixhawk serial port and must be
running first. On Graey both are systemd services (graey-mavproxy, graey-ros),
so this normally starts itself on boot; run it by hand only after
    sudo systemctl stop graey-ros

    ros2 launch robotx_graey_2026 core.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node

PKG = 'robotx_graey_2026'


def generate_launch_description():
    return LaunchDescription([
        Node(package=PKG, executable='led_node', name='led_node', output='screen'),
        Node(package=PKG, executable='pixhawk_led_node', name='pixhawk_led_node', output='screen'),
        Node(package=PKG, executable='dvl_node', name='dvl_node', output='screen'),
        Node(package=PKG, executable='vn100_node', name='vn100_node', output='screen'),
        Node(package=PKG, executable='nav_ekf_bridge', name='nav_ekf_bridge', output='screen'),
        Node(package=PKG, executable='gui_node', name='gui_node', output='screen'),
    ])
