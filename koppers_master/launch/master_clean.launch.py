"""
Team: Carnegie Mellon MRSD Team A: CreoClean
Members: David Hill, Louis Plottel, Leonardo Mouta, Michael Gromis, Yatharth Ahuja

File: master_clean.launch.py
Main Author: David Hill
Date: 2024-04-09

Description: Launch file for running koppers master cleaning node for SVD.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    node = Node(
        package='koppers_master',
        executable='clean_master'
    )
    ld.add_action(node)
    return ld