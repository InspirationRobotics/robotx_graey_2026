"""Graey core stack: status LEDs, kill switch, DVL/VN-100 navigation, and the GUI.

MAVProxy is NOT started here - it owns the Pixhawk serial port and must be
running first. On Graey both are systemd services (graey-mavproxy, graey-ros),
so this normally starts itself on boot; run it by hand only after
    sudo systemctl stop graey-ros

    ros2 launch robotx_graey_2026 core.launch.py

Sensor/control nodes respawn; the stateful navigation bridge deliberately does
not. Its position starts at zero, so recovery requires frame revalidation.
ros2 launch does NOT restart a dead node on its own, and
graey-ros.service only restarts when the launch PROCESS exits - so before this, one
node crashing left a permanent hole in the stack while everything else kept running.
That is how pixhawk_led_node stayed down after QGC had already recovered on
2026-08-07. Restart=always covers the service; this covers the nodes inside it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'robotx_graey_2026'
RESPAWN = {'respawn': True, 'respawn_delay': 2.0}


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('continuous_fusion', default_value='true',
            description='Pressure-gated external odometry; disabled pending commissioning'),
        DeclareLaunchArgument('surface_pressure_hpa', default_value='0.0',
            description='Measured confirmed-surface pressure for experimental bridge; zero disables depth'),
        DeclareLaunchArgument('odometry_enabled', default_value='false',
            description='Permit output only after automatic north alignment and sensor/profile checks'),
        DeclareLaunchArgument('mission_commissioned', default_value='false',
            description='Enable supervisor after live Cube fusion and mission checks'),
        DeclareLaunchArgument('waypoint_enabled', default_value='0',
            description='Set 1 to expose live waypoint execution after commissioning; readiness gates still apply'),
        Node(package=PKG, executable='led_node', name='led_node', output='screen', **RESPAWN),
        Node(package=PKG, executable='pixhawk_led_node', name='pixhawk_led_node', output='screen', **RESPAWN),
        # Belongs in the CORE stack, not the mission: SA must kill whether or not a
        # mission happens to be loaded.
        Node(package=PKG, executable='kill_switch', name='kill_switch', output='screen', **RESPAWN),
        Node(package=PKG, executable='dvl_node', name='dvl_node', output='screen', **RESPAWN),
        # Operator removed the temporary yaw correction on 2026-10-11.
        # Retain the separate physical roll mounting transform.
        Node(package=PKG, executable='vn100_node', name='vn100_node',
             parameters=[{'yaw_offset_deg': 0.0}], output='screen', **RESPAWN),
        Node(package=PKG, executable='fused_nav_bridge', name='nav_ekf_bridge',
             parameters=[{'enabled': ParameterValue(LaunchConfiguration('odometry_enabled'), value_type=bool),
                 'auto_north_alignment': True, 'require_disarmed_start': True,
                 'surface_pressure_hpa': ParameterValue(
                 LaunchConfiguration('surface_pressure_hpa'), value_type=float)}], output='screen', respawn=False),
        Node(package=PKG, executable='navigation_supervisor', name='navigation_supervisor',
             parameters=[{'active': ParameterValue(LaunchConfiguration('mission_commissioned'), value_type=bool),
                 'vehicle_validation_complete': ParameterValue(LaunchConfiguration('mission_commissioned'), value_type=bool),
                 'continuous_fusion': ParameterValue(
                 LaunchConfiguration('continuous_fusion'), value_type=bool)}], output='screen', **RESPAWN),
        Node(package=PKG, executable='gui_node', name='gui_node', output='screen',
             additional_env={'GRAEY_WAYPOINT_ENABLED': LaunchConfiguration('waypoint_enabled')}, **RESPAWN),
    ])

