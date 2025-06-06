from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='offboard_control_pkg',
            executable='offboard_control_node',
            name='control',
            output='screen'),
        Node(
            package='offboard_control_pkg',
            executable='Drone_One_Node',
            name='control',
            output='screen')
    ])