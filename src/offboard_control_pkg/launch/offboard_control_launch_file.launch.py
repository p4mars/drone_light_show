from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='offboard_control_pkg',
            executable='control_node',
            name='control_node',
            output='screen'),
        Node(
            package='offboard_control_pkg',
            executable='drone1',
            name='drone1',
            output='screen'),
        Node(
            package='offboard_control_pkg',
            executable='drone2',
            name='drone2',
            output='screen'),
        Node(
            package='offboard_control_pkg',
            executable='drone3',
            name='drone3',
            output='screen')
    ])