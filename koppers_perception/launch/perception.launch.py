"""
Team: Carnegie Mellon MRSD Team A: CreoClean
Members: David Hill, Louis Plottel, Leonardo Mouta, Michael Gromis, Yatharth Ahuja

File: perception.launch.py
Main Author: David Hill
Date: 2024-04-13

Description: Launch file for running perception node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    node = Node(
        package='koppers_perception',
        executable='process_image'
    )
    ld.add_action(node)
    return ld