from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='offboard_control_pkg',
            executable='Control_Node',
            name='control_node',
            output='screen'),
        Node(
            package='offboard_control_pkg',
            executable='Drone_One_Node',
            name='drone1_node',
            output='screen'),
        Node(
            package='offboard_control_pkg',
            executable='Drone_Two_Node',
            name='drone2_node',
            output='screen'),
        Node(
            package='offboard_control_pkg',
            executable='Drone_Three_Node',
            name='drone3_node',
            output='screen')
    ])