"""
Team: Carnegie Mellon MRSD Team A: CreoClean
Members: David Hill, Louis Plottel, Leonardo Mouta, Michael Gromis, Yatharth Ahuja

File: uf850_mover.py
Main Author: David Hill
Date: 2024-02-13

Description: ROS2 node for controlling the UF850 robot arm. This node listens for CleaningRequest messages and PoseRequest messages
"""

import rclpy
import time
import numpy as np
import quaternion
from rclpy.node import Node

from uf850_control.robot_library.transforms import *

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from xarm_msgs.srv import PlanJoint, PlanExec, PlanPose, PlanSingleStraight
from koppers_msgs.msg import CleaningRequest, PoseRequest
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from enum import Enum, auto

class State(Enum):
    CLEAN_CALLBACK = auto()
    PRE_CLEAN = auto()
    START_CLEAN = auto()
    MAIN_CLEAN = auto()
    STOP_CLEAN = auto()
    RETRACT_CLEAN = auto()
    MOVE_CALLBACK = auto()
    MOVE = auto()

class UF850Mover(Node):

    def __init__(self):
        super().__init__('uf850_mover')
        #clients for moveit services
        self.joint_plan_client = self.create_client(PlanJoint, '/xarm_joint_plan')
        self.pose_plan_client = self.create_client(PlanPose, '/xarm_pose_plan')
        self.straight_plan_client = self.create_client(PlanSingleStraight, '/xarm_straight_plan')
        self.exec_plan_client = self.create_client(PlanExec, '/xarm_exec_plan')
        #subscribers for listening to cleaning and moving requests
        self.clean_subscriber = self.create_subscription(CleaningRequest, '/uf850_clean', self.clean_callback, 10)
        self.move_subscriber = self.create_subscription(PoseRequest, '/uf850_move', self.move_callback, 10)
        #publishers for sending success messages
        self.clean_publisher = self.create_publisher(Bool, '/uf850_clean_success', 10)
        self.move_publisher = self.create_publisher(Bool, '/uf850_move_success', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #TODO: probably a better way than just putting the DH parameters here
        dh = [[0, 0.364, 0, np.pi/2.0],
            [np.pi/2.0, 0, 0.39, np.pi],
            [np.pi/2.0, 0, 0.15, -np.pi/2.0],
            [0, 0.426, 0, -np.pi/2.0],
            [0, 0, 0, np.pi/2.0],
            [0, 0.09, 0, 0]]

        self.squeegee_width = 0.15  # width of the squeegee in meters
        self.pre_clean_offset = np.array([0, 0, 0.01]) # offset for pre-cleaning position
        self.cleaning_trajs = None
        self.orientation = None

    def ik_uf850(self, target_position, target_rotation, mode=0):
        """
        Inverse kinematics function for the UF850 arm. Takes in a target position and rotation and returns the joint angles.
        """
        position = target_position

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

        #two options for the wrist rotation, choose based on custom heuristic
        if(mode == 0 and np.round(np.cos(q[0]),5) <= 0) or (mode == 1 and np.round(np.cos(q[0]),5) > 0):
            q[3] = np.arctan2(R_diff[1,2], R_diff[0,2])
            q[4] = np.arctan2(np.sqrt(R_diff[1,2]**2 + R_diff[0,2]**2), R_diff[2,2])
            q[5] = np.arctan2(R_diff[2,1], -R_diff[2,0])
        else:
            q[3] = np.arctan2(-R_diff[1,2], -R_diff[0,2])
            q[4] = np.arctan2(-np.sqrt(R_diff[1,2]**2 + R_diff[0,2]**2), R_diff[2,2])
            q[5] = np.arctan2(-R_diff[2,1], R_diff[2,0])

        return q
    
    def get_transform(self, id1, id2):
        """
        Get the transform between two frames
        """
        T = self.tf_buffer.lookup_transform(id1, id2, rclpy.time.Time())
        R = quaternion.as_rotation_matrix(quaternion.as_quat_array(np.array([T.transform.rotation.w,T.transform.rotation.x,T.transform.rotation.y,T.transform.rotation.z])))
        t = np.array([T.transform.translation.x, T.transform.translation.y, T.transform.translation.z])
        return R, t

    def clean_callback(self, msg):
        """
        Callback for cleaning requests
        """
        self.state = State.CLEAN_CALLBACK #set state to clean callback
        self.get_logger().info("In state " + str(self.state))

        self.start_position = np.array([msg.start_position.x, msg.start_position.y, msg.start_position.z])  # start position of the cleaning
        self.end_position = np.array([msg.end_position.x, msg.end_position.y, msg.end_position.z]) # end position of the cleaning
        #orientation of the cleaning plane
        self.cleaning_plane_orientation = quaternion.as_rotation_matrix(quaternion.as_quat_array(np.array([msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z])))
        self.mode = msg.mode # mode of the cleaning

        self.plan_pre_clean()   # plan the pre-cleaning motion

    def plan_pre_clean(self):
        """
        Plan the pre-cleaning motion
        """
        self.state = State.PRE_CLEAN #set state to pre-clean
        self.get_logger().info("In state " + str(self.state) + ", mode: " + str(self.mode))

        R_ee_6, t_ee_6 = self.get_transform('ee_mode_' + str(self.mode), 'link6')

        #calculate the pre-cleaning position and orientation
        pre_clean_position = self.start_position + self.cleaning_plane_orientation @ (self.pre_clean_offset + t_ee_6)
        pre_clean_orientation = self.cleaning_plane_orientation @ R_ee_6
        # pre_clean_orientation = quaternion.from_rotation_matrix(self.cleaning_plane_orientation @ R_ee_6)

        #calculate the joint angles for the pre-cleaning position and orientation
        q = self.ik_uf850(pre_clean_position, pre_clean_orientation, mode=self.mode)

        joint_request = PlanJoint.Request()
        joint_request.target = q.tolist()
        pre_clean_plan = self.joint_plan_client.call_async(joint_request)   #call the joint plan service
        pre_clean_plan.add_done_callback(self.moveit_callback)  #set the callback to the general moveit callback

        # pose_request = PlanPose.Request()
        # pose_request.target.position.x = pre_clean_position[0]
        # pose_request.target.position.y = pre_clean_position[1]
        # pose_request.target.position.z = pre_clean_position[2]
        # pose_request.target.orientation.x = pre_clean_orientation.x
        # pose_request.target.orientation.y = pre_clean_orientation.y
        # pose_request.target.orientation.z = pre_clean_orientation.z
        # pose_request.target.orientation.w = pre_clean_orientation.w

        # pre_clean_plan = self.pose_plan_client.call_async(pose_request)   #call the pose plan service
        # pre_clean_plan.add_done_callback(self.moveit_callback)


    def plan_start_clean(self):
        """
        Plan the start cleaning motion
        """
        self.state = State.START_CLEAN #set state to start clean
        self.get_logger().info("In state " + str(self.state) + ", mode: " + str(self.mode))

        R_ee_6, t_ee_6 = self.get_transform('ee_mode_' + str(self.mode), 'link6')

        #calculate the start cleaning position and orientation
        start_clean_position = self.start_position + self.cleaning_plane_orientation @ t_ee_6
        start_clean_orientation = quaternion.from_rotation_matrix(self.cleaning_plane_orientation @ R_ee_6)

        #create a request for the straight path planner
        straight_path_request = PlanSingleStraight.Request()
        start_clean_pose = Pose()
        start_clean_pose.position.x = start_clean_position[0]
        start_clean_pose.position.y = start_clean_position[1]
        start_clean_pose.position.z = start_clean_position[2]
        start_clean_pose.orientation.x = start_clean_orientation.x
        start_clean_pose.orientation.y = start_clean_orientation.y
        start_clean_pose.orientation.z = start_clean_orientation.z
        start_clean_pose.orientation.w = start_clean_orientation.w

        straight_path_request.target = [start_clean_pose]
        plan_future = self.straight_plan_client.call_async(straight_path_request)  #call the straight path plan service
        plan_future.add_done_callback(self.moveit_callback) #set the callback to the general moveit callback

    def plan_main_clean(self):
        """
        Plan the main cleaning motion
        """
        self.state = State.MAIN_CLEAN #set state to main clean
        self.get_logger().info("In state " + str(self.state) + ", mode: " + str(self.mode))

        R_ee_6, t_ee_6 = self.get_transform('ee_mode_' + str(self.mode), 'link6')

        #calculate the end cleaning position and orientation
        end_clean_position = self.end_position + self.cleaning_plane_orientation @ t_ee_6
        end_clean_orientation = quaternion.from_rotation_matrix(self.cleaning_plane_orientation @ R_ee_6)

        #create a request for the straight path planner
        straight_path_request = PlanSingleStraight.Request()
        end_clean_pose = Pose()
        end_clean_pose.position.x = end_clean_position[0]
        end_clean_pose.position.y = end_clean_position[1]
        end_clean_pose.position.z = end_clean_position[2]
        end_clean_pose.orientation.x = end_clean_orientation.x
        end_clean_pose.orientation.y = end_clean_orientation.y
        end_clean_pose.orientation.z = end_clean_orientation.z
        end_clean_pose.orientation.w = end_clean_orientation.w

        straight_path_request.target = [end_clean_pose]
        plan_future = self.straight_plan_client.call_async(straight_path_request) #call the straight path plan service
        plan_future.add_done_callback(self.moveit_callback) #set the callback to the general moveit callback

    def plan_stop_clean(self):
        """
        Plan the stop cleaning motion
        """
        self.state = State.STOP_CLEAN #set state to stop clean
        self.get_logger().info("In state " + str(self.state) + ", mode: " + str(self.mode))

        R_ee_6, t_ee_6 = self.get_transform('ee_mode_' + str(self.mode), 'link6')

        #calculate the stop cleaning position and orientation
        stop_clean_position = self.end_position + self.cleaning_plane_orientation @ (self.pre_clean_offset + t_ee_6)
        stop_clean_orientation = quaternion.from_rotation_matrix(self.cleaning_plane_orientation @ R_ee_6)

        #create a request for the straight path planner
        straight_path_request = PlanSingleStraight.Request()
        stop_clean_pose = Pose()
        stop_clean_pose.position.x = stop_clean_position[0]
        stop_clean_pose.position.y = stop_clean_position[1]
        stop_clean_pose.position.z = stop_clean_position[2]
        stop_clean_pose.orientation.x = stop_clean_orientation.x
        stop_clean_pose.orientation.y = stop_clean_orientation.y
        stop_clean_pose.orientation.z = stop_clean_orientation.z
        stop_clean_pose.orientation.w = stop_clean_orientation.w

        straight_path_request.target = [stop_clean_pose]
        plan_future = self.straight_plan_client.call_async(straight_path_request) #call the straight path plan service
        plan_future.add_done_callback(self.moveit_callback) #set the callback to the general moveit callback

    def plan_retract_clean(self):
        """
        Plan the retract cleaning motion
        """
        self.state = State.RETRACT_CLEAN #set state to retract clean
        self.get_logger().info("In state " + str(self.state) + ", mode: " + str(self.mode))

        R_ee_6, t_ee_6 = self.get_transform('ee_mode_' + str(self.mode), 'link6')

        #calculate the retract cleaning position and orientation
        pre_clean_position = self.start_position + self.cleaning_plane_orientation @ (self.pre_clean_offset + t_ee_6)
        pre_clean_orientation = quaternion.from_rotation_matrix(self.cleaning_plane_orientation @ R_ee_6)

        #create a request for the straight path planner
        straight_path_request = PlanSingleStraight.Request()
        pre_clean_pose = Pose()
        pre_clean_pose.position.x = pre_clean_position[0]
        pre_clean_pose.position.y = pre_clean_position[1]
        pre_clean_pose.position.z = pre_clean_position[2]
        pre_clean_pose.orientation.x = pre_clean_orientation.x
        pre_clean_pose.orientation.y = pre_clean_orientation.y
        pre_clean_pose.orientation.z = pre_clean_orientation.z
        pre_clean_pose.orientation.w = pre_clean_orientation.w

        straight_path_request.target = [pre_clean_pose]
        plan_future = self.straight_plan_client.call_async(straight_path_request) #call the straight path plan service
        plan_future.add_done_callback(self.moveit_callback) #set the callback to the general moveit callback

    def move_callback(self, msg):
        """
        Callback for moving requests
        """
        self.state = State.MOVE_CALLBACK #set state to move callback
        self.get_logger().info("In state " + str(self.state))

        self.desired_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) #desired position
        #desired orientation
        self.desired_orientation = quaternion.as_rotation_matrix(quaternion.as_quat_array(np.array([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])))
        self.reference_frame = msg.reference_frame #reference frame

        self.plan_move() #plan the move

    def plan_move(self):
        """
        Plan the move
        """
        self.state = State.MOVE #set state to move
        self.get_logger().info("In state " + str(self.state))

        R_ee_6, t_ee_6 = self.get_transform(self.reference_frame, 'link6')

        #calculate the position and orientation
        position = self.desired_position + self.desired_orientation @ t_ee_6
        orientation = self.desired_orientation @ R_ee_6

        #calculate the joint angles for the position and orientation
        q = self.ik_uf850(position, orientation)

        joint_request = PlanJoint.Request()
        joint_request.target = q.tolist()
        pre_clean_plan = self.joint_plan_client.call_async(joint_request)  #call the joint plan service
        pre_clean_plan.add_done_callback(self.moveit_callback) #set the callback to the general moveit callback
        self.latest_plan_executed = False

    def execute_latest_plan(self):
        """
        Execute the latest plan
        """
        self.get_logger().info("In execution")
        exec_request = PlanExec.Request()
        exec_request.wait = True
        exec_future = self.exec_plan_client.call_async(exec_request) #call the exec plan service
        exec_future.add_done_callback(self.moveit_callback) #set the callback to the general moveit callback

    def move_request_finished(self, success):
        """
        Function to publish the success of the move request
        """
        msg = Bool()
        msg.data = success
        self.move_publisher.publish(msg)

    def clean_request_finished(self, success):
        """
        Function to publish the success of the clean request
        """
        msg = Bool()
        msg.data = success
        self.clean_publisher.publish(msg)

    def moveit_callback(self, moveit_response):
        """
        Moveit callback function that directs the robot to the next state based on the current state
        """
        success = moveit_response.result().success
        #if the moveit service was successful
        if success:
            #if the last action was a successful execution, we will be planning next
            if self.latest_plan_executed:
                self.latest_plan_executed = False
                match self.state:
                    case State.PRE_CLEAN: #if the pre-clean motion was just executed, plan the start clean motion
                        self.plan_start_clean()
                    case State.START_CLEAN: #if the start clean motion was just executed, plan the main clean motion
                        self.plan_main_clean()
                    case State.MAIN_CLEAN: #if the main clean motion was just executed, plan the stop clean motion
                        self.plan_stop_clean()
                    case State.STOP_CLEAN: #if the stop clean motion was just executed
                        if self.mode == 0: #if we just pushed, the cleaning is done
                            self.clean_request_finished(True)
                        else:   #if we just pulled, plan the retract clean motion
                            self.plan_retract_clean()
                    case State.RETRACT_CLEAN: #if the retract clean motion was just executed, the cleaning is done
                        self.clean_request_finished(True)
                    case State.MOVE: #if the move was just executed, the move is done
                        self.move_request_finished(True)
            #if the last action was a successful planning, we will be executing next
            else:
                self.latest_plan_executed = True
                self.execute_latest_plan()
        #if the moveit service was not successful
        else:
            self.get_logger().info("moveit failed") #log that the moveit service failed
            #if the last action was an unsuccessful planning, we will try planning again
            if not self.latest_plan_executed:
                self.latest_plan_executed = False
                match self.state: #retry the plan that was just tried
                    case State.PRE_CLEAN:
                        self.plan_pre_clean()
                    case State.START_CLEAN:
                        self.plan_start_clean()
                    case State.MAIN_CLEAN:
                        self.plan_main_clean()
                    case State.STOP_CLEAN:
                        self.plan_stop_clean()
                    case State.RETRACT_CLEAN:
                        self.plan_retract_clean()
                    case State.MOVE:
                        self.plan_move()
            #if the last action was an unsuccessful execution, we will try executing again
            else:
                self.latest_plan_executed = True
                self.execute_latest_plan()

def main(args=None):
    rclpy.init(args=args)

    mover = UF850Mover() #create the UF850Mover node

    rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
