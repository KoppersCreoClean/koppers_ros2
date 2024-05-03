# koppers_ros2

## 1. Introduction

This repository is an collection of ROS2 packages containing all of the software for CMU MRSD Team A project CreoClean. We have been tasked with designing and building a robot to clean creosote off a steel drip pan surface. The corporate partner for this project is Koppers, a large company that works in wood treatment among other things.

## 2. Installing the Package

- ### 2.1 Install [ROS2 Humble](https://docs.ros.org/en/ros2_documentation/humble/Installation.html)
    This repository is only tested with ROS2 Humble, so please only use it with ROS2 Humble.

- ### 2.2 Install [Moveit2](https://moveit.ros.org/install-moveit2/binary/)  

- ### 2.3 Install [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
    Installing gazebo is only necessary for some unused UFactory packages to build. If you are willing to only build the neccessary packages for our application, you can skip installing Gazebo.

- ### 2.4 Install [gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)

- ### 2.5 Create a workspace
    ```bash
    # Skip this step if you already have a target workspace
    $ cd ~
    $ mkdir -p dev_ws/src
    ```

- ### 2.6 Obtain source code of "xarm_ros2" repository
    Make sure that your $ROS_DISTRO is humble. If the xarm_ros2 repository is giving you trouble, refer to this [link](https://github.com/xArm-Developer/xarm_ros2/blob/humble/ReadMe.md).
    ```bash
    # Remember to source ros2 environment settings first
    $ cd ~/dev_ws/src
    # DO NOT omit "--recursive"，or the source code of dependent submodule will not be downloaded.
    # Pay attention to the use of the -b parameter command branch, $ROS_DISTRO indicates the currently activated ROS version, if the ROS environment is not activated, you need to customize the specified branch (foxy/galactic/humble)
    $ git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO
    ```

- ### 2.7 Update "xarm_ros2" repository 
    ```bash
    $ cd ~/dev_ws/src/xarm_ros2
    $ git pull
    $ git submodule sync
    $ git submodule update --init --remote
    ```

- ### 2.8 Install dependencies
    ```bash
    # Remember to source ros2 environment settings first
    $ cd ~/dev_ws/src/
    $ rosdep update
    $ rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    ```

- ### 2.9 Obtain source code of "koppers_ros2" repository
    ```bash
    # Remember to source ros2 environment settings first
    $ cd ~/dev_ws/src

    $ git clone https://github.com/KoppersCreoClean/koppers_ros2.git
    ```

- ### 2.10 Build packages
    ```bash
    # Remember to source ros2 and moveit2 environment settings first
    $ cd ~/dev_ws/
    # build all packages
    $ colcon build

    # build selected packages
    $ colcon build --packages-select koppers_master
    ```

## 3. Using the Package

There are 5 packages in the repository. Below are brief descriptions and example uses for each of them.

- ### 3.1 koppers_msg
    This package defines messages and services needed by the rest of the packages. This is the only package that is a C++ package. The rest are python packages.
    

- ### 3.2 koppers_perception
    This package includes all of the nodes involved in perception, from capturing an image to detecting geometry and creosote. Right now, the camera is not defined using dev_rules. This should be changed eventually.

    ```bash
    # launch perception node to capture images and segmenting creosote
    $ ros2 launch koppers_perception perception.launch.py
    ```

- ### 3.3 drip_pan_environment
    This package includes the node used for updating the planning scene inside MoveIt based on the position of the linear actuator and mobility platform. It also adds the cleaning tool to the arm and enforces the squeegee does not move outside the area directly above the drip pan. This node relies on the UFactory ROS2 software and MoveIt.

    ```bash
    # launch file to start UFactory 850 arm in simulation
    $ ros2 launch xarm_planner uf850_planner_fake.launch.py

    # launch file to start UFactory 850 arm on hardware
    $ ros2 launch xarm_planner uf850_planner_realmove.launch.py robot_ip:=192.168.1.181

    # launch file to import testbed environment into the MoveIt scene
    $ ros2 launch drip_pan_environment testbed_scene.launch.py

    ```

- ### 3.4 uf850_control
    This package includes the node used for interfacing with MoveIt and the UFactory ROS2 package. When running, the node will recieve movement or cleaning commands and execute them using a custom inverse kinematics function, a planner native to MoveIt, and a state machine for breaking up commands into smaller steps.

    ```bash
    # launch file to start UFactory 850 arm in simulation
    $ ros2 launch xarm_planner uf850_planner_fake.launch.py

    # launch file to start UFactory 850 arm on hardware
    $ ros2 launch xarm_planner uf850_planner_realmove.launch.py robot_ip:=192.168.1.181

    # launch file to import testbed environment into the MoveIt scene
    $ ros2 launch drip_pan_environment testbed_scene.launch.py

    # launch node to interface with UFactory nodes
    $ ros2 launch uf850_control uf850_move.launch.py
    ```


- ### 3.5 koppers_master
    This package contains the master node. The master node is where decisions are made based on input from other nodes that describe the robot environment state as well as user input. The master node is not exactly a state machine, but it behaves in a similar way.


    ```bash
    # launch file to run the master node
    $ ros2 launch koppers_master master_clean.launch.py
    
    # launch file to run all the nodes needed for the robot to opperate
    $ ros2 launch koppers_master perception_manipulation.launch.py
    ```