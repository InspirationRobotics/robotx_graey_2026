"""Everything needed for a prequalification run, except MAVProxy and the mission.

Start MAVProxy FIRST in its own terminal (it owns the Pixhawk serial port):
    ./src/robotx_graey_2026/scripts/start_mavproxy.sh <LAPTOP_IP>

Then:
    ros2 launch robotx_graey_2026 prequal.launch.py

That brings up the CV pole tracker and position server, using the existing core
service. For standalone use AFTER stopping that service, pass start_core:=true.
The mission node is
deliberately NOT started here - run it by hand when you are ready:
    ros2 run robotx_graey_2026 prequal_mission_cv --ros-args -p dry_run:=false ...

Debug views:  camera  http://<jetson-ip>:8080     planner feed  :8081/pos
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

PKG = 'robotx_graey_2026'


def generate_launch_description():
    core = os.path.join(get_package_share_directory(PKG), 'launch', 'core.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('start_core', default_value='false',
                              description='Only true when graey-ros is stopped'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(core),
                                 condition=IfCondition(LaunchConfiguration('start_core'))),
        Node(package=PKG, executable='pole_tracker', name='pole_tracker', output='screen'),
        Node(package=PKG, executable='pos_server', name='pos_server', output='screen'),
    ])
