import rclpy
import time
import numpy as np
import quaternion
from rclpy.node import Node

from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger
from koppers_msgs.msg import CleaningRequest, PoseRequest
from koppers_msgs.srv import CreosoteSegment

class MasterNode(Node):

    def __init__(self):
        super().__init__('clean_master')
        self.linear_actuator_publisher = self.create_publisher(Float32, '/linear_actuator_position', 10)
        self.cart_publisher = self.create_publisher(Float32, '/cart_position', 10)
        self.scene_client = self.create_client(Trigger, '/update_scene')
        self.clean_publisher = self.create_publisher(CleaningRequest, '/uf850_clean', 10)
        self.pose_publisher = self.create_publisher(PoseRequest, '/uf850_move', 10)
        self.clean_subscriber = self.create_subscription(Bool, '/uf850_clean_success', self.clean_callback, 10)
        self.move_subscriber = self.create_subscription(Bool, '/uf850_move_success', self.move_callback, 10)
        self.perception_client = self.create_client(Trigger, '/capture_image')
        self.segmentation_client = self.create_client(CreosoteSegment, '/creosote_segmentation')

        self.update_scene()

        self.cleaning_trajs = np.array([[[-0.125,-0.48,-0.345],[-0.125,-0.34,-0.345]],[[0,-0.48,-0.345],[0,-0.34,-0.345]],[[0.125,-0.48,-0.345],[0.125,-0.34,-0.345]],[[-0.125,-0.34,-0.345],[-0.125,0,-0.355]],[[0,-0.34,-0.345],[0,0,-0.355]],[[0.125,-0.34,-0.345],[0.125,0,-0.355]]])
        self.cleaning_modes = np.array([0,0,0,1,1,1]).astype(int)
        self.squeegee_width = 0.15
        self.complete = False

    def update_scene(self):
        #move linear actuator and cart to desired position
        self.linear_actuator_publisher.publish(Float32(data=0.6))
        self.cart_publisher.publish(Float32(data=1.28))

        self.service_complete = False
        self.future = self.scene_client.call_async(Trigger.Request())
        self.future.add_done_callback(self.move_arm_to_camera_position)
        print("updating scene...")
        print("scene updated")

    def move_arm_to_camera_position(self, response):
        input("Press Enter to continue...")
        #move are to image capture position
        pose_request = PoseRequest()
        pose_request.pose.position.x = 0.0
        pose_request.pose.position.y = -0.29
        pose_request.pose.position.z = 0.18
        pose_request.pose.orientation.x = 0.7071068
        pose_request.pose.orientation.y = 0.7071068
        pose_request.pose.orientation.z = 0.0
        pose_request.pose.orientation.w = 0.0
        pose_request.reference_frame = "left_camera"
        self.pose_publisher.publish(pose_request)

        print("moving arm...")
        
    def capture_image(self):
        print("arm moved to image capture position")
        input("Press Enter to continue...")
        # get image
        self.service_complete = False
        self.future = self.perception_client.call_async(Trigger.Request())
        self.future.add_done_callback(self.segment_creosote)
        print("capturing image...")
        print("image captured")

    def segment_creosote(self, response):
        input("Press Enter to continue...")
        creosote_segment_request = CreosoteSegment.Request()
        x = self.cleaning_trajs[0,0,0]
        creosote_segment_request.x_min = x - self.squeegee_width / 2
        creosote_segment_request.x_max = x + self.squeegee_width / 2
        creosote_segment_request.y_min = self.cleaning_trajs[0,0,1]
        creosote_segment_request.y_max = self.cleaning_trajs[0,1,1]
        self.future = self.segmentation_client.call_async(creosote_segment_request)
        self.future.add_done_callback(self.clean_segment)
        print("segmenting creosote...")

    def clean_segment(self, response):
        print("creosote_segmented")
        y_min = response.result().y_min_result
        y_max = response.result().y_max_result
        if y_min < y_max:
            # print(y_min)
            cleaning_request = CleaningRequest()
            cleaning_request.start_position.x = self.cleaning_trajs[0,0,0]
            cleaning_request.start_position.y = y_min
            cleaning_request.start_position.z = self.cleaning_trajs[0,0,2]
            cleaning_request.end_position.x = self.cleaning_trajs[0,1,0]
            cleaning_request.end_position.y = self.cleaning_trajs[0,1,1]
            cleaning_request.end_position.z = self.cleaning_trajs[0,1,2]
            cleaning_request.orientation.x = -0.0061706
            cleaning_request.orientation.y = 0.0061706
            cleaning_request.orientation.z = 0.7070799
            cleaning_request.orientation.w = 0.7070799
            cleaning_request.mode = int(self.cleaning_modes[0])

            # print(cleaning_request)

            self.clean_publisher.publish(cleaning_request)
            self.cleaning_trajs = self.cleaning_trajs[1:,:,:]
            self.cleaning_modes = self.cleaning_modes[1:]
            print("cleaning segment...")
        else:
            self.cleaning_trajs = self.cleaning_trajs[1:,:,:]
            self.cleaning_modes = self.cleaning_modes[1:]
            print("no creosote found in segment")
            if self.cleaning_trajs.shape[0] > 0:
                self.segment_creosote(True)
            else:
                self.complete = True
                self.move_arm_to_camera_position(True)   

    def move_callback(self, msg):
        if self.complete:
            print("done cleaning")
        else:
            self.capture_image()

    def clean_callback(self, msg):
        if self.cleaning_modes.size > 0:
            print("segment cleaned")
            if self.cleaning_trajs.shape[0] > 0:
                self.segment_creosote(True)
            else:
                self.complete = True
                self.move_arm_to_camera_position(True)


def main(args=None):
    rclpy.init(args=args)

    node = MasterNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
