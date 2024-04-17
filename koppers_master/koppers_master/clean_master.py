import rclpy
import time
import numpy as np
import quaternion
from rclpy.node import Node

from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger
from koppers_msgs.msg import CleaningRequest, PoseRequest
from koppers_msgs.srv import CreosoteSegment, CreosoteImage
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32

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
        self.perception_client = self.create_client(CreosoteImage, '/capture_image')
        self.segmentation_client = self.create_client(CreosoteSegment, '/creosote_segmentation')
        self.point_cloud_subscriber = self.create_subscription(PointCloud, '/raw_creosote_point_cloud', self.point_cloud_callback, 10)
        self.dirty_point_cloud_publisher = self.create_publisher(PointCloud, '/dirty_creosote_point_cloud', 10)
        self.clean_point_cloud_publisher = self.create_publisher(PointCloud, '/clean_creosote_point_cloud', 10)

        self.update_scene()

        self.cleaning_trajs = np.array([[[-0.125,-0.53,-0.345],[-0.125,-0.39,-0.35]],[[0,-0.53,-0.345],[0,-0.39,-0.35]],[[0.125,-0.53,-0.345],[0.125,-0.39,-0.35]],[[-0.125,-0.39,-0.35],[-0.125,-0.05,-0.36]],[[0,-0.39,-0.35],[0,-0.05,-0.36]],[[0.125,-0.39,-0.35],[0.125,-0.05,-0.36]]])
        self.cleaning_modes = np.array([0,0,0,1,1,1]).astype(int)
        self.squeegee_width = 0.15
        self.x_min = -0.125 - self.squeegee_width / 2
        self.x_max = 0.125 + self.squeegee_width / 2
        self.y_min = -0.53
        self.y_max = -0.2
        self.complete = False
        self.dirty_point_cloud = []
        self.clean_point_cloud = []

    def publish_point_clouds(self):
        dirty_point_cloud_msg = PointCloud()
        dirty_point_cloud_msg.header.frame_id = 'link_base'
        dirty_point_cloud_msg.points = self.dirty_point_cloud
        for i in range(len(self.dirty_point_cloud)):
            dirty_point_cloud_msg.channels.append(ChannelFloat32(name="rgb",values=[0,0,0]))
        dirty_point_cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.dirty_point_cloud_publisher.publish(dirty_point_cloud_msg)

        clean_point_cloud_msg = PointCloud()
        clean_point_cloud_msg.header.frame_id = 'link_base'
        clean_point_cloud_msg.points = self.clean_point_cloud
        for i in range(len(self.clean_point_cloud)):
            clean_point_cloud_msg.channels.append(ChannelFloat32(name="rgb",values=[255,255,255]))
        clean_point_cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.clean_point_cloud_publisher.publish(clean_point_cloud_msg)

    def point_cloud_callback(self, msg):
        self.dirty_point_cloud = msg.points
        self.clean_point_cloud = []
        # x_min = np.min(np.min(self.cleaning_trajs,0),0)[0]
        # x_max = np.max(np.max(self.cleaning_trajs,0),0)[0]
        # y_min = np.min(np.min(self.cleaning_trajs,0),0)[1]
        # y_max = np.max(np.max(self.cleaning_trajs,0),0)[1] - 0.1
        
        # for point in msg.points:
        #     if point.x >= x_min - self.squeegee_width / 2 and point.x <= x_max + self.squeegee_width / 2 and point.y >= y_min and point.y <= y_max:
        #         self.dirty_point_cloud.append(point)
        
        self.publish_point_clouds()

    def update_point_clouds(self, x, y1, y2):
        i = 0
        y_min = min(y1,y2)
        y_max = max(y1,y2)
        while i < len(self.dirty_point_cloud):
            point = self.dirty_point_cloud[i]
            if point.x >= x - self.squeegee_width / 2 and point.x <= x + self.squeegee_width / 2 and point.y >= y_min and point.y <= y_max:
                self.clean_point_cloud.append(point)
                self.dirty_point_cloud = self.dirty_point_cloud[:i] + self.dirty_point_cloud[i+1:]
            else:
                i = i+1
        self.publish_point_clouds()    

    def clear_clean_points(self):        
        self.clean_point_cloud = []
        self.publish_point_clouds()    

    def update_scene(self):
        #move linear actuator and cart to desired position
        self.linear_actuator_publisher.publish(Float32(data=0.55))
        self.cart_publisher.publish(Float32(data=1.28))

        self.service_complete = False
        self.future = self.scene_client.call_async(Trigger.Request())
        self.future.add_done_callback(self.move_arm_to_camera_position)
        self.get_logger().info("updating scene...")
        self.get_logger().info("scene updated")

    def move_arm_to_camera_position(self, response):
        # input("Press Enter to continue...")
        #move are to image capture position
        pose_request = PoseRequest()
        pose_request.pose.position.x = 0.0
        pose_request.pose.position.y = -0.34
        pose_request.pose.position.z = 0.18
        pose_request.pose.orientation.x = 0.7071068
        pose_request.pose.orientation.y = 0.7071068
        pose_request.pose.orientation.z = 0.0
        pose_request.pose.orientation.w = 0.0
        pose_request.reference_frame = "left_camera"
        self.pose_publisher.publish(pose_request)

        self.get_logger().info("moving arm...")
        
    def capture_image(self):
        self.get_logger().info("arm moved to image capture position")
        # input("Press Enter to continue...")
        # get image
        self.service_complete = False
        image_request = CreosoteImage.Request()
        image_request.x_min = self.x_min
        image_request.x_max = self.x_max
        image_request.y_min = self.y_min
        image_request.y_max = self.y_max
        self.future = self.perception_client.call_async(image_request)
        self.future.add_done_callback(self.segment_creosote)
        self.get_logger().info("capturing image...")
        self.get_logger().info("image captured")

    def segment_creosote(self, response):
        self.get_logger().info("segmenting creosote...")
        creosote_segment_request = CreosoteSegment.Request()
        x = self.cleaning_trajs[0,0,0]
        creosote_segment_request.x_min = x - self.squeegee_width / 2
        creosote_segment_request.x_max = x + self.squeegee_width / 2
        creosote_segment_request.y_min = self.cleaning_trajs[0,0,1]
        creosote_segment_request.y_max = self.cleaning_trajs[0,1,1]
        self.future = self.segmentation_client.call_async(creosote_segment_request)
        self.future.add_done_callback(self.clean_segment)

    def clean_segment(self, response):
        y_min = response.result().y_min_result
        y_max = response.result().y_max_result
        self.get_logger().info(f'y_min: {y_min}')
        self.get_logger().info(f'y_max: {y_max}')
        if y_min < y_max:
            # self.get_logger().info(y_min)
            k = (self.cleaning_trajs[0,1,1] - y_min) / (self.cleaning_trajs[0,1,1] - self.cleaning_trajs[0,0,1])
            cleaning_request = CleaningRequest()
            cleaning_request.start_position.x = self.cleaning_trajs[0,0,0]
            cleaning_request.start_position.y = y_min
            cleaning_request.start_position.z = k * self.cleaning_trajs[0,0,2] + (1 - k) * self.cleaning_trajs[0,1,2]
            cleaning_request.end_position.x = self.cleaning_trajs[0,1,0]
            cleaning_request.end_position.y = self.cleaning_trajs[0,1,1]
            cleaning_request.end_position.z = self.cleaning_trajs[0,1,2]
            cleaning_request.orientation.x = -0.0061706
            cleaning_request.orientation.y = 0.0061706
            cleaning_request.orientation.z = 0.7070799
            cleaning_request.orientation.w = 0.7070799
            cleaning_request.mode = int(self.cleaning_modes[0])

            # self.get_logger().info(cleaning_request)

            self.update_point_clouds(self.cleaning_trajs[0,0,0], y_min, self.cleaning_trajs[0,1,1])
            time.sleep(5)

            self.clean_publisher.publish(cleaning_request)
            self.cleaning_trajs = self.cleaning_trajs[1:,:,:]
            self.cleaning_modes = self.cleaning_modes[1:]
            self.get_logger().info("cleaning segment...")
        else:
            self.cleaning_trajs = self.cleaning_trajs[1:,:,:]
            self.cleaning_modes = self.cleaning_modes[1:]
            self.get_logger().info("no creosote found in segment")
            if self.cleaning_trajs.shape[0] > 0:
                self.segment_creosote(True)
            else:
                self.complete = True
                self.move_arm_to_camera_position(True)   

    def move_callback(self, msg):
        if self.complete:
            self.get_logger().info("done cleaning")
        else:
            self.capture_image()

    def clean_callback(self, msg):
        self.get_logger().info("segment cleaned")
        self.clear_clean_points()
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
