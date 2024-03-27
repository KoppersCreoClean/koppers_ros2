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
    PRE_CLEAN_1 = auto()
    START_CLEAN_1 = auto()
    MAIN_CLEAN_1 = auto()
    STOP_CLEAN_1 = auto()
    PRE_CLEAN_2 = auto()
    START_CLEAN_2 = auto()
    MAIN_CLEAN_2 = auto()
    STOP_CLEAN_2 = auto()

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
        self.cleaning_trajs = None
        self.orientation = None
        self.state = State.SPLIT_CLEANING_AREA
        self.latest_plan_executed = False

    def get_transform(self, id1, id2):
        T = self.tf_buffer.lookup_transform(id1, id2, rclpy.time.Time())
        R = quaternion.as_rotation_matrix(quaternion.as_quat_array(np.array([T.transform.rotation.w,T.transform.rotation.x,T.transform.rotation.y,T.transform.rotation.z])))
        t = np.array([T.transform.translation.x, T.transform.translation.y, T.transform.translation.z])
        return R, t

    def clean_callback(self, msg):
        corners = np.array([[msg.corner_1.x,msg.corner_1.y,msg.corner_1.z], [msg.corner_2.x,msg.corner_2.y,msg.corner_2.z], [msg.corner_3.x,msg.corner_3.y,msg.corner_3.z], [msg.corner_4.x,msg.corner_4.y,msg.corner_4.z]])
        self.cleaning_plane_orientation = quaternion.as_rotation_matrix(quaternion.as_quat_array(np.array([msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z])))
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

    def develop_segment_strategy(self):
        self.state = State.DEVELOP_SEGMENT_STRATEGY
        self.get_logger().info("In state ", self.state)
        self.step_dist = 0.01
        self.target_traj = self.cleaning_trajs[0]
        self.strategy_iterator = 0
        self.strategy_mode = 1
        self.target_cleaning_points = [self.target_traj[0] + n * 0.01 * (self.target_traj[1] - self.target_traj[0]) / norm(self.target_traj[1] - self.target_traj[0]) for n in range(int(np.floor(norm(self.target_traj[1] - self.target_traj[0])/self.step_dist)))]
        self.target_cleaning_points.append(self.target_traj[1])
        self.mode_1_results = np.zeros(len(self.target_cleaning_points))
        self.mode_2_results = np.zeros(len(self.target_cleaning_points))
        self.strategy_result = None
        self.iterate_segment_strategy()

    def iterate_segment_strategy(self):
        if self.strategy_result is not None:
            if self.strategy_mode == 1:
                self.mode_1_results[self.strategy_iterator] = self.strategy_result
            else:
                self.mode_2_results[self.strategy_iterator] = self.strategy_result
            
            if self.strategy_iterator < len(self.target_cleaning_points) - 1:
                self.strategy_iterator = self.strategy_iterator + 1
            elif self.strategy_mode == 1:
                self.strategy_mode = 2
                self.strategy_iterator = 0
            elif self.strategy_mode == 2:
                self.complete_segment_strategy()

        if self.strategy_mode == 1:
            R_ee_6, t_ee_6 = self.get_transform('ee_mode_1', 'link6')
        elif self.strategy_mode == 2:
            R_ee_6, t_ee_6 = self.get_transform('ee_mode_2', 'link6')

        test_position = self.target_cleaning_points[self.strategy_iterator] + self.cleaning_plane_orientation @ (self.pre_clean_offset + t_ee_6)
        test_orientation = quaternion.from_rotation_matrix(self.cleaning_plane_orientation @ R_ee_6)

        pose_request = PlanPose.Request()
        pose_request.target.position.x = test_position[0]
        pose_request.target.position.y = test_position[1]
        pose_request.target.position.z = test_position[2]
        pose_request.target.orientation.w = test_orientation.w
        pose_request.target.orientation.x = test_orientation.x
        pose_request.target.orientation.y = test_orientation.y
        pose_request.target.orientation.z = test_orientation.z

        test_plan = self.pose_plan_client.call_async(pose_request)
        test_plan.add_done_callback(self.moveit_callback)

    def complete_segment_strategy(self):
        mode_1_start = None
        mode_2_start = None
        mode_1_end = None
        mode_2_end = None
        for i in range(len(self.target_cleaning_points)):
            if mode_1_start is None and self.mode_1_results[i] == 1:
                mode_1_start = i
            if mode_2_start is None and self.mode_2_results[i] == 1:
                mode_2_start = i
            if mode_1_start is not None and self.mode_1_results[i] == 0:
                mode_1_end = i
            if mode_2_start is not None and self.mode_2_results[i] == 0:
                mode_2_end = i

        if mode_1_start == 0 and mode_1_end == len(self.target_cleaning_points) - 1:
            pass
        elif mode_2_start == 0 and mode_2_end == len(self.target_cleaning_points) - 1:
            pass
        elif mode_1_start < mode_2_start and mode_1_end > mode_2_end:
            pass
        elif mode_1_start > mode_2_start and mode_1_end < mode_2_end:
            pass
        elif mode_1_start < mode_2_start and mode_1_end < mode_2_end:
            pass
        elif mode_1_start > mode_2_start and mode_1_end > mode_2_end:
            pass
        

    def plan_pre_clean(self, mode, state):
        self.state = state
        self.get_logger().info("In state ", self.state)

        traj = self.cleaning_trajs[0]
        start_position = traj[0]
        pre_clean_position = start_position + self.orientation @ self.pre_clean_offset

        q = self.ik_uf850(pre_clean_position, self.orientation)
        joint_request = PlanJoint.Request()
        joint_request.target = q.tolist()
        plan_future = self.joint_plan_client.call_async(joint_request)
        plan_future.add_done_callback(self.moveit_callback)

    def plan_start_clean(self, mode, state):
        self.state = state
        self.get_logger().info("In state ", self.state)

        traj = self.cleaning_trajs[0]
        start_position = traj[0]
        start_orientation = quaternion.from_rotation_matrix(self.orientation)

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

    def plan_main_clean(self, mode, state):
        self.state = state
        self.get_logger().info("In state ", self.state)

        traj = self.cleaning_trajs[0]
        end_position = traj[1]
        end_orientation = quaternion.from_rotation_matrix(self.orientation)

        self.start_time = time.time()
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

    def plan_end_clean(self, mode, state):
        self.state = state
        self.get_logger().info("In state ", self.state)

        traj = self.cleaning_trajs[0]
        end_position = traj[1]
        post_clean_position = end_position + self.orientation @ self.pre_clean_offset
        post_clean_orientation = quaternion.from_rotation_matrix(self.orientation)

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
        plan_future.add_done_callback(self.moveit_callback)

    def execute_latest_plan(self):
        exec_request = PlanExec.Request()
        exec_request.wait = True
        exec_future = self.exec_plan_client.call_async(exec_request)
        exec_future.add_done_callback(self.moveit_callback)

    def moveit_callback(self, moveit_response):
        success = moveit_response.result().success
        if self.state == State.DEVELOP_SEGMENT_STRATEGY:
            self.strategy_result = success
            self.iterate_segment_strategy()
        if success:
            if self.latest_plan_executed:
                match self.state:
                    case State.SPLIT_CLEANING_AREA:
                        pass
                    case State.PRE_CLEAN_1:
                        self.plan_start_clean(mode=1, state=State.START_CLEAN_1)
                    case State.START_CLEAN_1:
                        self.plan_main_clean(mode=1, state=State.MAIN_CLEAN_1)
                    case State.MAIN_CLEAN_1:
                        self.plan_end_clean(mode=1, state=State.STOP_CLEAN_1)
                    case State.STOP_CLEAN_1:
                        self.plan_pre_clean(mode=2, state=State.PRE_CLEAN_2)
                    case State.PRE_CLEAN_2:
                        self.plan_start_clean(mode=2, state=State.START_CLEAN_2)
                    case State.START_CLEAN_2:
                        self.plan_main_clean(mode=2, state=State.MAIN_CLEAN_2)
                    case State.MAIN_CLEAN_2:
                        self.plan_end_clean(mode=2, state=State.STOP_CLEAN_2)
                    case State.STOP_CLEAN_2:
                        pass
                self.latest_plan_executed = False
            else:
                self.execute_latest_plan()
                self.latest_plan_executed = True
        else:
            self.get_logger().info("moveit failed")

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
