from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    drones = ['px4_1', 'px4_2']

    return LaunchDescription([
        Node(
            package='offboard_control_pkg',
            executable='offboard_control_node',
            namespace=drone,
            name=f'{drone}_offboard',
            output='screen'
        ) for drone in drones
    ])