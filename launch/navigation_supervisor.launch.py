"""Observation launch; configure calibrated pressure before interpreting depth."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='robotx_graey_2026',
        executable='navigation_supervisor', name='navigation_supervisor',
        output='screen', parameters=[{'active': False}])])
