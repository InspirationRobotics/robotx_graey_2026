"""Opt-in competition reporting. MAVProxy/core must already be running."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=os.path.join(get_package_share_directory('robotx_graey_2026'),'config','graey_params.yaml')),
        DeclareLaunchArgument('ocs_host',default_value='auto',description='Laptop IPv4/hostname on the shared Wi-Fi router'),
        DeclareLaunchArgument('ocs_port',default_value='37564'),
        DeclareLaunchArgument('outbox_path',default_value='/root/robotx_ws/logs/competition-outbox.sqlite3'),
        Node(package='robotx_graey_2026',executable='competition_link_node',
             name='competition_link_node',output='screen',
             parameters=[LaunchConfiguration('params_file'),{'ocs_host':LaunchConfiguration('ocs_host'),
                          'ocs_port':ParameterValue(LaunchConfiguration('ocs_port'),value_type=int),
                          'outbox_path':LaunchConfiguration('outbox_path')}]),
    ])
