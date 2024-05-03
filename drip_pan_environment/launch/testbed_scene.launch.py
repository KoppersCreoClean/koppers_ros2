"""
Team: Carnegie Mellon MRSD Team A: CreoClean
Members: David Hill, Louis Plottel, Leonardo Mouta, Michael Gromis, Yatharth Ahuja

File: testbed_scene.launch.py
Main Author: David Hill
Date: 2024-02-24

Description: ROS2 launch file for running the arm scene interface on the testbed.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(get_package_share_directory('drip_pan_environment'),'config','testbed.yaml')
        
    node=Node(
        package='drip_pan_environment',
        executable='arm_scene_interface',
        parameters = [config]
    )
    ld.add_action(node)
    return ld