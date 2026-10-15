"""Local SITL only: real Graey bridge, supervisor and GUI plus synthetic inputs."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    home_lat = LaunchConfiguration('home_lat')
    home_lon = LaunchConfiguration('home_lon')
    north = LaunchConfiguration('waypoint_north_m')
    east = LaunchConfiguration('waypoint_east_m')
    result_file = str(Path(__file__).resolve().parents[1] / 'sitl' / 'logs' / 'scenario-result.json')
    return LaunchDescription([
        DeclareLaunchArgument('interactive_waypoint', default_value='false'),
        DeclareLaunchArgument('continuous_fusion', default_value='true'),
        DeclareLaunchArgument('surface_pressure_hpa', default_value='1013.25'),
        DeclareLaunchArgument('home_lat', default_value='32.9236125'),
        DeclareLaunchArgument('home_lon', default_value='-117.0385235'),
        DeclareLaunchArgument('waypoint_north_m', default_value='1.5'),
        DeclareLaunchArgument('waypoint_east_m', default_value='1.0'),
        DeclareLaunchArgument('supervisor_mavlink', default_value='udpout:127.0.0.1:14557'),
        DeclareLaunchArgument('result_file', default_value=result_file),
        Node(package='robotx_graey_2026', executable='fused_nav_bridge',
             additional_env={'GRAEY_RUNTIME_SCOPE': 'sitl'}, name='nav_ekf_bridge',
             parameters=[{'enabled': True, 'heading_reference_validated': True,
                          'sitl_depth_topic': '/graey/sitl/depth_m',
                          'surface_pressure_hpa': ParameterValue(LaunchConfiguration('surface_pressure_hpa'), value_type=float)}], output='screen'),
        Node(package='robotx_graey_2026', executable='navigation_supervisor',
             additional_env={'GRAEY_RUNTIME_SCOPE': 'sitl'},
             name='navigation_supervisor', parameters=[{
                 'active': True, 'vehicle_validation_complete': True,
                 'continuous_fusion': True,
                 'heading_reference_validated': True,  # synthetic sensors share true-NED axes
                 'mavlink': LaunchConfiguration('supervisor_mavlink'),
                                  'reference_file': '/tmp/graey-sitl-navigation-reference.json'}], output='screen'),
        Node(package='robotx_graey_2026', executable='sitl_sensor_sim',
             parameters=[{'truth_only': True}],
             name='sitl_sensor_sim', output='screen'),
        Node(package='robotx_graey_2026', executable='sitl_mission',
             condition=UnlessCondition(LaunchConfiguration('interactive_waypoint')),
             name='sitl_mission', parameters=[{
                 'home_lat': ParameterValue(home_lat, value_type=float),
                 'home_lon': ParameterValue(home_lon, value_type=float),
                 'waypoint_north_m': ParameterValue(north, value_type=float),
                 'waypoint_east_m': ParameterValue(east, value_type=float),
                 'result_file': LaunchConfiguration('result_file')}], output='screen'),
        Node(package='robotx_graey_2026', executable='gui_node', name='gui_node', output='screen'),
    ])
