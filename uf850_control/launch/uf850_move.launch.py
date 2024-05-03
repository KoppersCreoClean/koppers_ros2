"""
Team: Carnegie Mellon MRSD Team A: CreoClean
Members: David Hill, Louis Plottel, Leonardo Mouta, Michael Gromis, Yatharth Ahuja

File: uf850_move.launch.py
Main Author: David Hill
Date: 2024-02-25

Description: Launch file for the UFactory 850 arm mover node.
"""


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    node = Node(
        package='uf850_control',
        executable='mover'
    )
    ld.add_action(node)
    return ld