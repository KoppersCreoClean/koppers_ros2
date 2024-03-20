# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import time
import numpy as np
import quaternion
from rclpy.node import Node

from uf850_control.robot_library.kinematics import SerialArm
from uf850_control.robot_library.transforms import *

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from xarm_msgs.srv import PlanJoint, PlanExec, PlanPose, PlanSingleStraight
from koppers_msgs.msg import RectangularCleanArea


class UF850Mover(Node):

    def __init__(self):
        super().__init__('uf850_mover')
        self.joint_plan_client = self.create_client(PlanJoint, '/xarm_joint_plan')
        self.straight_plan_client = self.create_client(PlanSingleStraight, '/xarm_straight_plan')
        self.exec_plan_client = self.create_client(PlanExec, '/xarm_exec_plan')
        self.clean_subscriber = self.create_subscription(RectangularCleanArea, '/clean_area', self.clean_callback, 10)
        self.clean_publisher = self.create_publisher(Bool, '/clean_success', 10)

        #TODO: probably a better way than just putting the DH parameters here
        dh = [[0, 0.364, 0, np.pi/2.0],
            [np.pi/2.0, 0, 0.39, np.pi],
            [np.pi/2.0, 0, 0.15, -np.pi/2.0],
            [0, 0.426, 0, -np.pi/2.0],
            [0, 0, 0, np.pi/2.0],
            [0, 0.09, 0, 0]]
        
        self.arm = SerialArm(dh)
        self.squeegee_width = 0.15
        self.pre_clean_offset = np.array([0, 0, -0.01])
        self.cleaning_trajs = None
        self.orientation = None

    def ik_uf850(self, target_position, target_rotation):
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
        q[1] = q1 + q1_offset
        q[2] = q2_offset - q2

        raw_rotation = rotz(q[0]) @ roty(q[2]-q[1]) @ np.diag([1,-1,-1])

        R_diff = raw_rotation.T @ target_rotation

        if(abs(np.arctan2(R_diff[1,2], R_diff[0,2])) < abs(np.arctan2(-R_diff[1,2], -R_diff[0,2]))):
            q[3] = np.arctan2(R_diff[1,2], R_diff[0,2])
            q[4] = np.arctan2(np.sqrt(R_diff[1,2]**2 + R_diff[0,2]**2), R_diff[2,2])
            q[5] = np.arctan2(R_diff[2,1], -R_diff[2,0])
        else:
            q[3] = np.arctan2(-R_diff[1,2], -R_diff[0,2])
            q[4] = np.arctan2(-np.sqrt(R_diff[1,2]**2 + R_diff[0,2]**2), R_diff[2,2])
            q[5] = np.arctan2(-R_diff[2,1], R_diff[2,0])

        return q

    def clean_callback(self, msg):
        corners = np.array([[msg.corner_1.x,msg.corner_1.y,msg.corner_1.z], [msg.corner_2.x,msg.corner_2.y,msg.corner_2.z], [msg.corner_3.x,msg.corner_3.y,msg.corner_3.z], [msg.corner_4.x,msg.corner_4.y,msg.corner_4.z]])
        self.orientation = quaternion.as_rotation_matrix(quaternion.as_quat_array(np.array([msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z])))
        print(self.orientation)
        width = np.sqrt(np.sum(np.square(corners[0,:] - corners[1,:])))
        height = np.sqrt(np.sum(np.square(corners[0,:] - corners[3,:])))
        num_passes = int(np.ceil(width/self.squeegee_width))
        clean_offset = (self.squeegee_width - (num_passes * self.squeegee_width - width)) / 2
        self.cleaning_trajs = []
        for i in range(num_passes):
            start_pos = corners[0,:] * (num_passes - i) / num_passes + corners[1,:] * i / num_passes + clean_offset * (corners[1,:] - corners[0,:]) / width
            end_pos = corners[3,:] * (num_passes - i) / num_passes + corners[2,:] * i / num_passes + clean_offset * (corners[1,:] - corners[0,:]) / width
            self.cleaning_trajs.append(np.array([start_pos, end_pos]))
        print(self.cleaning_trajs)
        print("beginning cleaning section...")
        self.clean_plan_segment(None)

    def clean_plan_segment(self, exec_response):
        if exec_response is None:
            print(self.cleaning_trajs[0])
            self.plan_pre_clean()
        else:
            success = exec_response.result().success
            print("execution success? " + str(success))
            if success and len(self.cleaning_trajs) == 0:
                self.clean_publisher.publish(Bool(data=True))
            elif success:
                print(self.cleaning_trajs[0])
                self.plan_pre_clean()
            else:
                self.clean_publisher.publish(Bool(data=False))

    def plan_pre_clean(self):
        traj = self.cleaning_trajs[0]
        start_position = traj[0]
        pre_clean_position = start_position + self.orientation @ self.pre_clean_offset

        q = self.ik_uf850(pre_clean_position, self.orientation)

        print("planning pre_clean...")
        joint_request = PlanJoint.Request()
        joint_request.target = q.tolist()
        plan_future = self.joint_plan_client.call_async(joint_request)
        plan_future.add_done_callback(self.exec_pre_clean)
    
    def exec_pre_clean(self, plan_response):
        success = plan_response.result().success
        print("planning success? " + str(success))
        if success:
            print("executing pre_clean...")
            exec_request = PlanExec.Request()
            exec_request.wait = True
            exec_future = self.exec_plan_client.call_async(exec_request)
            exec_future.add_done_callback(self.plan_start_clean)
        else:
            self.clean_publisher.publish(Bool(data=False))

    def plan_start_clean(self, exec_response):
        success = exec_response.result().success
        print("execution success? " + str(success))
        if success:
            traj = self.cleaning_trajs[0]
            start_position = traj[0]
            start_orientation = quaternion.from_rotation_matrix(self.orientation)

            print("planning start_clean...")
            straight_path_request = PlanSingleStraight.Request()
            start_pose = Pose()
            start_pose.position.x = start_position[0]
            start_pose.position.y = start_position[1]
            start_pose.position.z = start_position[2]
            start_pose.orientation.x = start_orientation.x
            start_pose.orientation.y = start_orientation.y
            start_pose.orientation.z = start_orientation.z
            start_pose.orientation.w = start_orientation.w

            straight_path_request.target = [start_pose]
            plan_future = self.straight_plan_client.call_async(straight_path_request)
            plan_future.add_done_callback(self.exec_start_clean)
        else:
            self.clean_publisher.publish(Bool(data=False))

    def exec_start_clean(self, plan_response):
        success = plan_response.result().success
        print("planning success? " + str(success))
        if success:
            print("executing start_clean...")
            exec_request = PlanExec.Request()
            exec_request.wait = True
            exec_future = self.exec_plan_client.call_async(exec_request)
            exec_future.add_done_callback(self.plan_clean)
        else:
            self.clean_publisher.publish(Bool(data=False))

    def plan_clean(self, exec_response):
        success = exec_response.result().success
        print("execution success? " + str(success))
        if success:
            traj = self.cleaning_trajs[0]
            end_position = traj[1]
            end_orientation = quaternion.from_rotation_matrix(self.orientation)

            print("planning clean...")
            straight_path_request = PlanSingleStraight.Request()
            end_pose = Pose()
            end_pose.position.x = end_position[0]
            end_pose.position.y = end_position[1]
            end_pose.position.z = end_position[2]
            end_pose.orientation.x = end_orientation.x
            end_pose.orientation.y = end_orientation.y
            end_pose.orientation.z = end_orientation.z
            end_pose.orientation.w = end_orientation.w

            straight_path_request.target = [end_pose]
            plan_future = self.straight_plan_client.call_async(straight_path_request)
            plan_future.add_done_callback(self.exec_clean)
        else:
            self.clean_publisher.publish(Bool(data=False))

    def exec_clean(self, plan_response):
        success = plan_response.result().success
        print("planning success? " + str(success))
        if success:
            print("executing clean...")
            exec_request = PlanExec.Request()
            exec_request.wait = True
            exec_future = self.exec_plan_client.call_async(exec_request)
            exec_future.add_done_callback(self.plan_end_clean)
        else:
            self.clean_publisher.publish(Bool(data=False))

    def plan_end_clean(self, exec_response):
        success = exec_response.result().success
        print("execution success? " + str(success))
        if success:
            traj = self.cleaning_trajs[0]
            end_position = traj[1]
            post_clean_position = end_position + self.orientation @ self.pre_clean_offset
            post_clean_orientation = quaternion.from_rotation_matrix(self.orientation)

            print("planning end_clean...")
            straight_path_request = PlanSingleStraight.Request()
            post_clean_pose = Pose()
            post_clean_pose.position.x = post_clean_position[0]
            post_clean_pose.position.y = post_clean_position[1]
            post_clean_pose.position.z = post_clean_position[2]
            post_clean_pose.orientation.x = post_clean_orientation.x
            post_clean_pose.orientation.y = post_clean_orientation.y
            post_clean_pose.orientation.z = post_clean_orientation.z
            post_clean_pose.orientation.w = post_clean_orientation.w

            straight_path_request.target = [post_clean_pose]
            plan_future = self.straight_plan_client.call_async(straight_path_request)
            plan_future.add_done_callback(self.exec_end_clean)
        else:
            self.clean_publisher.publish(Bool(data=False))

    def exec_end_clean(self, plan_response):
        success = plan_response.result().success
        print("planning success? " + str(success))
        if success:
            print("executing end_clean...")
            exec_request = PlanExec.Request()
            exec_request.wait = True
            exec_future = self.exec_plan_client.call_async(exec_request)
            self.cleaning_trajs.pop(0)
            exec_future.add_done_callback(self.clean_plan_segment)
        else:
            self.clean_publisher.publish(Bool(data=False))

def main(args=None):
    rclpy.init(args=args)

    mover = UF850Mover()

    rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
