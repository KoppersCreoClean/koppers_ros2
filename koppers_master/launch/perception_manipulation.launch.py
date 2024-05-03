"""
Team: Carnegie Mellon MRSD Team A: CreoClean
Members: David Hill, Louis Plottel, Leonardo Mouta, Michael Gromis, Yatharth Ahuja

File: perception_manipulation.launch.py
Main Author: David Hill
Date: 2024-04-13

Description: Launch file for running all required nodes for SVD except for the master node.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    #launch the uf850 simulator
    uf850_simulator_node= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_planner'), 'launch', 'uf850_planner_fake.launch.py']))
    )    
    ld.add_action(uf850_simulator_node)

    #launch the testbed environment
    testbed_environment_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('drip_pan_environment'), 'launch', 'testbed_scene.launch.py']))
    )    
    ld.add_action(testbed_environment_node)

    #launch the manipulation node
    movement_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('uf850_control'), 'launch', 'uf850_move.launch.py']))
    )    
    ld.add_action(movement_node)

    #launch the perception node
    perception_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('koppers_perception'), 'launch', 'perception.launch.py']))
    )    
    ld.add_action(perception_node)

    return ld