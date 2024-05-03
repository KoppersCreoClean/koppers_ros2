import rclpy
import time
import numpy as np
import quaternion
from rclpy.node import Node

from uf850_control.robot_library.kinematics import SerialArm
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
        self.joint_plan_client = self.create_client(PlanJoint, '/xarm_joint_plan')
        self.pose_plan_client = self.create_client(PlanPose, '/xarm_pose_plan')
        self.straight_plan_client = self.create_client(PlanSingleStraight, '/xarm_straight_plan')
        self.exec_plan_client = self.create_client(PlanExec, '/xarm_exec_plan')
        self.clean_subscriber = self.create_subscription(CleaningRequest, '/uf850_clean', self.clean_callback, 10)
        self.move_subscriber = self.create_subscription(PoseRequest, '/uf850_move', self.move_callback, 10)
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
        
        self.arm = SerialArm(dh)
        self.squeegee_width = 0.15
        self.pre_clean_offset = np.array([0, 0, 0.01])
        self.cleaning_trajs = None
        self.orientation = None

    def ik_uf850(self, target_position, target_rotation, mode=0):
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
        T = self.tf_buffer.lookup_transform(id1, id2, rclpy.time.Time())
        R = quaternion.as_rotation_matrix(quaternion.as_quat_array(np.array([T.transform.rotation.w,T.transform.rotation.x,T.transform.rotation.y,T.transform.rotation.z])))
        t = np.array([T.transform.translation.x, T.transform.translation.y, T.transform.translation.z])
        return R, t

    def clean_callback(self, msg):
        self.state = State.CLEAN_CALLBACK
        self.get_logger().info("In state " + str(self.state))

        self.start_position = np.array([msg.start_position.x, msg.start_position.y, msg.start_position.z])
        self.end_position = np.array([msg.end_position.x, msg.end_position.y, msg.end_position.z])
        self.cleaning_plane_orientation = quaternion.as_rotation_matrix(quaternion.as_quat_array(np.array([msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z])))
        self.mode = msg.mode

        self.plan_pre_clean()

    def plan_pre_clean(self):
        self.state = State.PRE_CLEAN
        self.get_logger().info("In state " + str(self.state) + ", mode: " + str(self.mode))

        R_ee_6, t_ee_6 = self.get_transform('ee_mode_' + str(self.mode), 'link6')

        pre_clean_position = self.start_position + self.cleaning_plane_orientation @ (self.pre_clean_offset + t_ee_6)
        pre_clean_orientation = self.cleaning_plane_orientation @ R_ee_6
        # pre_clean_orientation = quaternion.from_rotation_matrix(self.cleaning_plane_orientation @ R_ee_6)

        q = self.ik_uf850(pre_clean_position, pre_clean_orientation, mode=self.mode)

        joint_request = PlanJoint.Request()
        joint_request.target = q.tolist()
        pre_clean_plan = self.joint_plan_client.call_async(joint_request)
        pre_clean_plan.add_done_callback(self.moveit_callback)

        # pose_request = PlanPose.Request()
        # pose_request.target.position.x = pre_clean_position[0]
        # pose_request.target.position.y = pre_clean_position[1]
        # pose_request.target.position.z = pre_clean_position[2]
        # pose_request.target.orientation.x = pre_clean_orientation.x
        # pose_request.target.orientation.y = pre_clean_orientation.y
        # pose_request.target.orientation.z = pre_clean_orientation.z
        # pose_request.target.orientation.w = pre_clean_orientation.w

        # pre_clean_plan = self.pose_plan_client.call_async(pose_request)
        # pre_clean_plan.add_done_callback(self.moveit_callback)


    def plan_start_clean(self):
        self.state = State.START_CLEAN
        self.get_logger().info("In state " + str(self.state) + ", mode: " + str(self.mode))

        R_ee_6, t_ee_6 = self.get_transform('ee_mode_' + str(self.mode), 'link6')

        start_clean_position = self.start_position + self.cleaning_plane_orientation @ t_ee_6
        start_clean_orientation = quaternion.from_rotation_matrix(self.cleaning_plane_orientation @ R_ee_6)

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
        plan_future = self.straight_plan_client.call_async(straight_path_request)
        plan_future.add_done_callback(self.moveit_callback)

    def plan_main_clean(self):
        self.state = State.MAIN_CLEAN
        self.get_logger().info("In state " + str(self.state) + ", mode: " + str(self.mode))

        R_ee_6, t_ee_6 = self.get_transform('ee_mode_' + str(self.mode), 'link6')

        end_clean_position = self.end_position + self.cleaning_plane_orientation @ t_ee_6
        end_clean_orientation = quaternion.from_rotation_matrix(self.cleaning_plane_orientation @ R_ee_6)

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
        plan_future = self.straight_plan_client.call_async(straight_path_request)
        plan_future.add_done_callback(self.moveit_callback)

    def plan_stop_clean(self):
        self.state = State.STOP_CLEAN
        self.get_logger().info("In state " + str(self.state) + ", mode: " + str(self.mode))

        R_ee_6, t_ee_6 = self.get_transform('ee_mode_' + str(self.mode), 'link6')

        stop_clean_position = self.end_position + self.cleaning_plane_orientation @ (self.pre_clean_offset + t_ee_6)
        stop_clean_orientation = quaternion.from_rotation_matrix(self.cleaning_plane_orientation @ R_ee_6)

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
        plan_future = self.straight_plan_client.call_async(straight_path_request)
        plan_future.add_done_callback(self.moveit_callback)

    def plan_retract_clean(self):
        self.state = State.RETRACT_CLEAN
        self.get_logger().info("In state " + str(self.state) + ", mode: " + str(self.mode))

        R_ee_6, t_ee_6 = self.get_transform('ee_mode_' + str(self.mode), 'link6')

        pre_clean_position = self.start_position + self.cleaning_plane_orientation @ (self.pre_clean_offset + t_ee_6)
        pre_clean_orientation = quaternion.from_rotation_matrix(self.cleaning_plane_orientation @ R_ee_6)

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
        plan_future = self.straight_plan_client.call_async(straight_path_request)
        plan_future.add_done_callback(self.moveit_callback)

    def move_callback(self, msg):
        self.state = State.MOVE_CALLBACK
        self.get_logger().info("In state " + str(self.state))

        self.desired_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.desired_orientation = quaternion.as_rotation_matrix(quaternion.as_quat_array(np.array([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])))
        self.reference_frame = msg.reference_frame

        self.plan_move()

    def plan_move(self):
        self.state = State.MOVE
        self.get_logger().info("In state " + str(self.state))

        R_ee_6, t_ee_6 = self.get_transform(self.reference_frame, 'link6')

        position = self.desired_position + self.desired_orientation @ t_ee_6
        orientation = self.desired_orientation @ R_ee_6

        q = self.ik_uf850(position, orientation)

        joint_request = PlanJoint.Request()
        joint_request.target = q.tolist()
        pre_clean_plan = self.joint_plan_client.call_async(joint_request)
        pre_clean_plan.add_done_callback(self.moveit_callback)
        self.latest_plan_executed = False

    def execute_latest_plan(self):
        self.get_logger().info("In execution")
        exec_request = PlanExec.Request()
        exec_request.wait = True
        exec_future = self.exec_plan_client.call_async(exec_request)
        exec_future.add_done_callback(self.moveit_callback)

    def move_request_finished(self, success):
        msg = Bool()
        msg.data = success
        self.move_publisher.publish(msg)

    def clean_request_finished(self, success):
        msg = Bool()
        msg.data = success
        self.clean_publisher.publish(msg)

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
                        if self.mode == 0:
                            self.clean_request_finished(True)
                        else:
                            self.plan_retract_clean()
                    case State.RETRACT_CLEAN:
                        self.clean_request_finished(True)
                    case State.MOVE:
                        self.move_request_finished(True)
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
                    case State.RETRACT_CLEAN:
                        self.plan_retract_clean()
                    case State.MOVE:
                        self.plan_move()
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
