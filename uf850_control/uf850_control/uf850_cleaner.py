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
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from enum import Enum, auto

class State(Enum):
    SPLIT_CLEANING_AREA = auto()
    DEVELOP_SEGMENT_STRATEGY = auto()
    PRE_CLEAN = auto()
    START_CLEAN = auto()
    MAIN_CLEAN = auto()
    STOP_CLEAN = auto()

class UF850Mover(Node):

    def __init__(self):
        super().__init__('uf850_mover')
        self.joint_plan_client = self.create_client(PlanJoint, '/xarm_joint_plan')
        self.pose_plan_client = self.create_client(PlanPose, '/xarm_pose_plan')
        self.straight_plan_client = self.create_client(PlanSingleStraight, '/xarm_straight_plan')
        self.exec_plan_client = self.create_client(PlanExec, '/xarm_exec_plan')
        self.clean_subscriber = self.create_subscription(RectangularCleanArea, '/clean_area', self.clean_callback, 10)
        self.clean_publisher = self.create_publisher(Bool, '/clean_success', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.squeegee_width = 0.15
        self.pre_clean_offset = np.array([0, 0, 0.01])

    def get_transform(self, id1, id2):
        T = self.tf_buffer.lookup_transform(id1, id2, rclpy.time.Time())
        R = quaternion.as_rotation_matrix(quaternion.as_quat_array(np.array([T.transform.rotation.w,T.transform.rotation.x,T.transform.rotation.y,T.transform.rotation.z])))
        t = np.array([T.transform.translation.x, T.transform.translation.y, T.transform.translation.z])
        return R, t
    
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

        # if(abs(np.arctan2(R_diff[1,2], R_diff[0,2])) < abs(np.arctan2(-R_diff[1,2], -R_diff[0,2]))):
        q[3] = np.arctan2(R_diff[1,2], R_diff[0,2])
        q[4] = np.arctan2(np.sqrt(R_diff[1,2]**2 + R_diff[0,2]**2), R_diff[2,2])
        q[5] = np.arctan2(R_diff[2,1], -R_diff[2,0])
        # else:
        #     q[3] = np.arctan2(-R_diff[1,2], -R_diff[0,2])
        #     q[4] = np.arctan2(-np.sqrt(R_diff[1,2]**2 + R_diff[0,2]**2), R_diff[2,2])
        #     q[5] = np.arctan2(-R_diff[2,1], R_diff[2,0])

        return q

    def clean_callback(self, msg):
        self.state = State.SPLIT_CLEANING_AREA
        self.get_logger().info("In state " + str(self.state))

        corners = np.array([[msg.corner_1.x,msg.corner_1.y,msg.corner_1.z], [msg.corner_2.x,msg.corner_2.y,msg.corner_2.z], [msg.corner_3.x,msg.corner_3.y,msg.corner_3.z], [msg.corner_4.x,msg.corner_4.y,msg.corner_4.z]])
        self.cleaning_plane_orientation = quaternion.as_rotation_matrix(quaternion.as_quat_array(np.array([msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z])))
        width = np.sqrt(np.sum(np.square(corners[0,:] - corners[1,:])))
        height = np.sqrt(np.sum(np.square(corners[0,:] - corners[3,:])))
        num_passes = int(np.ceil(width/self.squeegee_width))
        clean_offset = (self.squeegee_width - (num_passes * self.squeegee_width - width)) / 2
        self.cleaning_trajs = []
        self.cleaning_modes = []
        for i in range(num_passes):
            start_pos = corners[0,:] * (num_passes - i) / num_passes + corners[1,:] * i / num_passes + clean_offset * (corners[1,:] - corners[0,:]) / width
            end_pos = corners[3,:] * (num_passes - i) / num_passes + corners[2,:] * i / num_passes + clean_offset * (corners[1,:] - corners[0,:]) / width
            mid_pos = 0.6 * start_pos + 0.4 * end_pos
            self.cleaning_trajs.append(np.array([start_pos, mid_pos]))
            self.cleaning_modes.append(0)
            self.cleaning_trajs.append(np.array([mid_pos, end_pos]))
            self.cleaning_modes.append(1)
        self.clean_segment()

    def clean_segment(self):
        if len(self.cleaning_trajs) == 0:
            self.clean_publisher.publish(Bool(data=True))
        else:
            self.latest_plan_executed = False
            self.plan_pre_clean()

    def plan_pre_clean(self,):
        self.get_logger().info("In state " + str(self.cleaning_trajs[0]))
        traj = self.cleaning_trajs[0]
        start_position = traj[0]
        mode = self.cleaning_modes[0]
        self.state = State.PRE_CLEAN
        self.get_logger().info("In state " + str(self.state) + ", mode: " + str(mode))

        R_ee_6, t_ee_6 = self.get_transform('ee_mode_' + str(mode), 'link6')

        pre_clean_position = start_position + self.cleaning_plane_orientation @ (self.pre_clean_offset + t_ee_6)
        pre_clean_orientation = self.cleaning_plane_orientation @ R_ee_6

        q = self.ik_uf850(pre_clean_position, pre_clean_orientation)

        joint_request = PlanJoint.Request()
        joint_request.target = q.tolist()
        pre_clean_plan = self.joint_plan_client.call_async(joint_request)
        pre_clean_plan.add_done_callback(self.moveit_callback)

    def plan_start_clean(self):
        traj = self.cleaning_trajs[0]
        start_position = traj[0]
        mode = self.cleaning_modes[0]
        self.state = State.START_CLEAN
        self.get_logger().info("In state " + str(self.state) + ", mode: " + str(mode))

        R_ee_6, t_ee_6 = self.get_transform('ee_mode_' + str(mode), 'link6')

        start_position = start_position + self.cleaning_plane_orientation @ t_ee_6
        start_orientation = quaternion.from_rotation_matrix(self.cleaning_plane_orientation @ R_ee_6)


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
        plan_future.add_done_callback(self.moveit_callback)

    def plan_main_clean(self):
        traj = self.cleaning_trajs[0]
        end_position = traj[1]
        mode = self.cleaning_modes[0]
        self.state = State.MAIN_CLEAN
        self.get_logger().info("In state " + str(self.state) + ", mode: " + str(mode))

        R_ee_6, t_ee_6 = self.get_transform('ee_mode_' + str(mode), 'link6')

        end_position = end_position + self.cleaning_plane_orientation @ t_ee_6
        end_orientation = quaternion.from_rotation_matrix(self.cleaning_plane_orientation @ R_ee_6)

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
        plan_future.add_done_callback(self.moveit_callback)

    def plan_stop_clean(self):
        traj = self.cleaning_trajs[0]
        end_position = traj[1]
        mode = self.cleaning_modes[0]
        self.state = State.STOP_CLEAN
        self.get_logger().info("In state " + str(self.state) + ", mode: " + str(mode))

        R_ee_6, t_ee_6 = self.get_transform('ee_mode_' + str(mode), 'link6')

        stop_position = end_position + self.cleaning_plane_orientation @ (self.pre_clean_offset + t_ee_6)
        stop_orientation = quaternion.from_rotation_matrix(self.cleaning_plane_orientation @ R_ee_6)

        straight_path_request = PlanSingleStraight.Request()
        start_pose = Pose()
        start_pose.position.x = stop_position[0]
        start_pose.position.y = stop_position[1]
        start_pose.position.z = stop_position[2]
        start_pose.orientation.x = stop_orientation.x
        start_pose.orientation.y = stop_orientation.y
        start_pose.orientation.z = stop_orientation.z
        start_pose.orientation.w = stop_orientation.w

        straight_path_request.target = [start_pose]
        plan_future = self.straight_plan_client.call_async(straight_path_request)
        plan_future.add_done_callback(self.moveit_callback)

        self.cleaning_trajs.pop(0)
        self.cleaning_modes.pop(0)


    def execute_latest_plan(self):
        self.get_logger().info("In execution")
        exec_request = PlanExec.Request()
        exec_request.wait = True
        exec_future = self.exec_plan_client.call_async(exec_request)
        exec_future.add_done_callback(self.moveit_callback)

    def moveit_callback(self, moveit_response):
        success = moveit_response.result().success
        if success:
            if self.latest_plan_executed:
                self.latest_plan_executed = False
                match self.state:
                    case State.PRE_CLEAN:
                        self.plan_start_clean()
                    case State.START_CLEAN:
                        self.plan_main_clean()
                    case State.MAIN_CLEAN:
                        self.plan_stop_clean()
                    case State.STOP_CLEAN:
                        self.clean_segment()
            else:
                self.latest_plan_executed = True
                self.execute_latest_plan()
        else:
            self.get_logger().info("moveit failed")
            if not self.latest_plan_executed:
                self.latest_plan_executed = False
                match self.state:
                    case State.PRE_CLEAN:
                        self.plan_pre_clean()
                    case State.START_CLEAN:
                        self.plan_start_clean()
                    case State.MAIN_CLEAN:
                        self.plan_main_clean()
                    case State.STOP_CLEAN:
                        self.plan_stop_clean()
            else:
                self.latest_plan_executed = True
                self.execute_latest_plan()

def main(args=None):
    rclpy.init(args=args)

    mover = UF850Mover()

    rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
