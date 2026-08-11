"""Graey core stack: status LEDs, kill switch, DVL/VN-100 navigation, and the GUI.

MAVProxy is NOT started here - it owns the Pixhawk serial port and must be
running first. On Graey both are systemd services (graey-mavproxy, graey-ros),
so this normally starts itself on boot; run it by hand only after
    sudo systemctl stop graey-ros

    ros2 launch robotx_graey_2026 core.launch.py

Every node respawns. ros2 launch does NOT restart a dead node on its own, and
graey-ros.service only restarts when the launch PROCESS exits - so before this, one
node crashing left a permanent hole in the stack while everything else kept running.
That is how pixhawk_led_node stayed down after QGC had already recovered on
2026-08-07. Restart=always covers the service; this covers the nodes inside it.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

PKG = 'robotx_graey_2026'
RESPAWN = {'respawn': True, 'respawn_delay': 2.0}


def generate_launch_description():
    return LaunchDescription([
        Node(package=PKG, executable='led_node', name='led_node', output='screen', **RESPAWN),
        Node(package=PKG, executable='pixhawk_led_node', name='pixhawk_led_node', output='screen', **RESPAWN),
        # Belongs in the CORE stack, not the mission: SA must kill whether or not a
        # mission happens to be loaded.
        Node(package=PKG, executable='kill_switch', name='kill_switch', output='screen', **RESPAWN),
        Node(package=PKG, executable='dvl_node', name='dvl_node', output='screen', **RESPAWN),
        Node(package=PKG, executable='vn100_node', name='vn100_node', output='screen', **RESPAWN),
        Node(package=PKG, executable='nav_ekf_bridge', name='nav_ekf_bridge', output='screen', **RESPAWN),
        Node(package=PKG, executable='gui_node', name='gui_node', output='screen', **RESPAWN),
    ])
