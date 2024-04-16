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