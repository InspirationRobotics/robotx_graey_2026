"""Local SITL only: real Graey bridge, supervisor and GUI plus synthetic inputs."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    home_lat = LaunchConfiguration('home_lat')
    home_lon = LaunchConfiguration('home_lon')
    north = LaunchConfiguration('waypoint_north_m')
    east = LaunchConfiguration('waypoint_east_m')
    result_file = str(Path(__file__).resolve().parents[1] / 'sitl' / 'logs' / 'scenario-result.json')
    return LaunchDescription([
        DeclareLaunchArgument('home_lat', default_value='32.9240586'),
        DeclareLaunchArgument('home_lon', default_value='-117.0385389'),
        DeclareLaunchArgument('waypoint_north_m', default_value='1.5'),
        DeclareLaunchArgument('waypoint_east_m', default_value='1.0'),
        Node(package='robotx_graey_2026', executable='nav_ekf_bridge',
             name='nav_ekf_bridge', parameters=[{'allow_alignment': True}], output='screen'),
        Node(package='robotx_graey_2026', executable='navigation_supervisor',
             name='navigation_supervisor', parameters=[{
                 'active': True, 'vehicle_validation_complete': True,
                 'depth_topic': '/graey/sitl/depth_m',
                 'reference_file': '/tmp/graey-sitl-navigation-reference.json'}], output='screen'),
        Node(package='robotx_graey_2026', executable='sitl_sensor_sim',
             name='sitl_sensor_sim', output='screen'),
        Node(package='robotx_graey_2026', executable='sitl_mission',
             name='sitl_mission', parameters=[{
                 'home_lat': ParameterValue(home_lat, value_type=float),
                 'home_lon': ParameterValue(home_lon, value_type=float),
                 'waypoint_north_m': ParameterValue(north, value_type=float),
                 'waypoint_east_m': ParameterValue(east, value_type=float),
                 'result_file': result_file}], output='screen'),
        Node(package='robotx_graey_2026', executable='gui_node', name='gui_node', output='screen'),
    ])
