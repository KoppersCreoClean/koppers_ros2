from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()
    uf850_simulator_node= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_planner'), 'launch', 'uf850_planner_fake.launch.py']))
    )    
    ld.add_action(uf850_simulator_node)

    testbed_environment_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('drip_pan_environment'), 'launch', 'testbed_scene.launch.py']))
    )    
    ld.add_action(testbed_environment_node)

    movement_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('uf850_control'), 'launch', 'uf850_move.launch.py']))
    )    
    ld.add_action(movement_node)

    perception_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('koppers_perception'), 'launch', 'perception.launch.py']))
    )    
    ld.add_action(perception_node)

    return ld