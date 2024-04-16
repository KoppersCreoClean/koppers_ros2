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