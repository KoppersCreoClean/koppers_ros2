"""
Team: Carnegie Mellon MRSD Team A: CreoClean
Members: David Hill, Louis Plottel, Leonardo Mouta, Michael Gromis, Yatharth Ahuja

File: master_clean.launch.py
Main Author: David Hill
Date: 2024-01-10

Description: Script for testing the inverse-kinematics function. Must run 'ros2 launch xarm_description
uf850_rviz_display.launch.py' before running this script for visualization.
"""

import rclpy
import time
import numpy as np
from rclpy.node import Node

from uf850_control.robot_library.kinematics import SerialArm
from uf850_control.robot_library.transforms import *

from sensor_msgs.msg import JointState


class JointStatePublisher(Node):

    def __init__(self, test_points):
        super().__init__('kinematics_publisher')
        #directly publish joint states
        self.publisher_ = self.create_publisher(JointState, '/ufactory/joint_states', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)   #publish new joint states every 0.1 seconds
        self.test_points = test_points
        self.i = 0

    def timer_callback(self):
        if(self.i < len(self.test_points)):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ["joint1","joint2","joint3","joint4","joint5","joint6"]
            msg.position = self.test_points[self.i]
            msg.velocity = [0.0,0.0,0.0,0.0,0.0,0.0]
            msg.effort = [0.0,0.0,0.0,0.0,0.0,0.0]
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.position)
            self.i += 1
    
def ik_uf850(target_position, rotation):
    """
    Inverse kinematics function for the UF850 arm. Takes in a target position and rotation and returns the joint angles.
    """
    position = target_position
    target_rotation = rotation

    q = np.zeros(6)

    position = position - target_rotation[:,2] * 0.09 - np.array([0,0,0.364])

    q[0] = np.arctan2(position[1],position[0])
    r = np.sqrt(position[0]**2 + position[1]**2)
    a1 = 0.39
    a2 = np.sqrt(0.15**2 + 0.426**2)
    q1_offset =  - np.pi/2
    q2_offset = -(np.pi/2 + np.arctan(0.426/0.15))
    q2 = -np.arccos((r**2 + position[2]**2 - a1**2 - a2**2)/(2*a1*a2))
    q1 = np.arctan2(position[2],r) - np.arctan2(a2*np.sin(q2),(a1 + a2*np.cos(q2)))
    #elbow up
    q[1] = q1 + q1_offset
    q[2] = q2_offset - q2

    raw_rotation = rotz(q[0]) @ roty(q[2]-q[1]) @ np.diag([1,-1,-1])

    R_diff = raw_rotation.T @ target_rotation
    
    #two options for the wrist rotation
    if(abs(np.arctan2(R_diff[1,2], R_diff[0,2])) > abs(np.arctan2(-R_diff[1,2], -R_diff[0,2]))):
        q[3] = np.arctan2(R_diff[1,2], R_diff[0,2])
        q[4] = np.arctan2(np.sqrt(R_diff[1,2]**2 + R_diff[0,2]**2), R_diff[2,2])
        q[5] = np.arctan2(R_diff[2,1], -R_diff[2,0])
        # print("a")
    else:
        q[3] = np.arctan2(-R_diff[1,2], -R_diff[0,2])
        q[4] = np.arctan2(-np.sqrt(R_diff[1,2]**2 + R_diff[0,2]**2), R_diff[2,2])
        q[5] = np.arctan2(-R_diff[2,1], R_diff[2,0])
        # print("b")

    return q

def main(args=None):
    rclpy.init(args=args)

    dh = [[0, 0.364, 0, np.pi/2.0],
          [np.pi/2.0, 0, 0.39, np.pi],
          [np.pi/2.0, 0, 0.15, -np.pi/2.0],
          [0, 0.426, 0, -np.pi/2.0],
          [0, 0, 0, np.pi/2.0],
          [0, 0.09, 0, 0]]  # DH parameters for the UF850 arm
    
    arm = SerialArm(dh) # Create a SerialArm object with the DH parameters

    test_points = []
    rot = np.diag([1,-1,-1]) #@ np.array([[0.939, 0.343, -0.0344], [-0.342, 0.94, 0.00606], [0.0344, 0.00606, 0.999]])

    ### LINES

    x_steps = 100
    y_steps = 100
    z_steps = 5

    ts = time.time()

    #make a grid of points to test
    for k in range(z_steps):
        for i in range(x_steps):
            for j in range(y_steps):
                tran = np.array([1.5*i/x_steps - 0.75, 1.5*j/y_steps - 0.75, k/z_steps - 0.3])
                q = ik_uf850(tran, rot)

                if not np.any(np.isnan(q)):
                    test_points.append(list(q))

                    pos = arm.fk(np.array(q))
                    # print(pos - np.vstack((np.hstack((rot, tran.reshape(3,1))), np.array([0,0,0,1]))))
                    if np.linalg.norm(pos - np.vstack((np.hstack((rot, tran.reshape(3,1))), np.array([0,0,0,1])))) > 1e-6:
                        print(np.linalg.norm(pos - np.vstack((np.hstack((rot, tran.reshape(3,1))), np.array([0,0,0,1])))))

    # print((time.time() - ts)/(x_steps*y_steps*z_steps))

    ### SPIRAL

    # num_steps = 100
    # num_rotations = 4
    # max_radius = 0.8
    # min_height = -0.5
    # max_height = 0.3
    
    # for i in range(num_steps+1):
    #     tran = np.array([np.cos(2*num_rotations*np.pi*i/num_steps)*(i/num_steps * max_radius),np.sin(2*num_rotations*np.pi*i/num_steps)*(i/num_steps * max_radius),(i/num_steps * (max_height - min_height) + min_height)])
    #     q = ik_uf850(tran, rot)

    #     if not np.any(np.isnan(q)):
    #         test_points.append(list(q))


    minimal_publisher = JointStatePublisher(test_points) # Create a JointStatePublisher object with the test points

    rclpy.spin(minimal_publisher)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
