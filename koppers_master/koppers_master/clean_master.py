"""
Team: Carnegie Mellon MRSD Team A: CreoClean
Members: David Hill, Louis Plottel, Leonardo Mouta, Michael Gromis, Yatharth Ahuja

File: clean_master.py
Main Author: David Hill
Date: 2024-04-02

Description: ROS2 node for running the state machine master in SVD.
"""

import rclpy
import time
import numpy as np
import quaternion
from rclpy.node import Node

from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from koppers_msgs.msg import CleaningRequest, PoseRequest
from koppers_msgs.srv import CreosoteSegment, CreosoteImage
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32

class MasterNode(Node):

    def __init__(self):
        super().__init__('clean_master')
        # publishers and clients for interacting with the arm scene interface
        self.linear_actuator_publisher = self.create_publisher(Float32, '/linear_actuator_position', 10)
        self.cart_publisher = self.create_publisher(Float32, '/cart_position', 10)
        self.scene_client = self.create_client(Trigger, '/update_scene')
        # publishers and clients for interacting with the arm
        self.clean_publisher = self.create_publisher(CleaningRequest, '/uf850_clean', 10)
        self.pose_publisher = self.create_publisher(PoseRequest, '/uf850_move', 10)
        self.clean_subscriber = self.create_subscription(Bool, '/uf850_clean_success', self.clean_callback, 10)
        self.move_subscriber = self.create_subscription(Bool, '/uf850_move_success', self.move_callback, 10)
        # publishers and clients for interacting with the perception pipeline
        self.perception_client = self.create_client(CreosoteImage, '/capture_image')
        self.segmentation_client = self.create_client(CreosoteSegment, '/creosote_segmentation')
        self.point_cloud_subscriber = self.create_subscription(PointCloud, '/raw_creosote_point_cloud', self.point_cloud_callback, 10)
        self.dirty_point_cloud_publisher = self.create_publisher(PointCloud, '/dirty_creosote_point_cloud', 10)
        self.clean_point_cloud_publisher = self.create_publisher(PointCloud, '/clean_creosote_point_cloud', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.update_scene()

        # self.cleaning_trajs = np.array([[[-0.125,-0.53,-0.345],[-0.125,-0.39,-0.35]],[[0,-0.53,-0.345],[0,-0.39,-0.35]],[[0.125,-0.53,-0.345],[0.125,-0.39,-0.35]],[[-0.125,-0.39,-0.35],[-0.125,-0.05,-0.36]],[[0,-0.39,-0.35],[0,-0.05,-0.36]],[[0.125,-0.39,-0.35],[0.125,-0.05,-0.36]]])
        self.cleaning_trajs = np.array([[[0.44,-0.4,-0.365],[0.44,-0.1,-0.37]],[[0.44,-0.12,-0.37],[0.44,0.4,-0.375]]])
        # arm_z_rotation = -135/180*np.pi
        # arm_z_T = np.array([[np.cos(arm_z_rotation), -np.sin(arm_z_rotation), 0, 0.0],
        #               [np.sin(arm_z_rotation), np.cos(arm_z_rotation), 0, 0.0],
        #               [0, 0, 1, 0.0],
        #               [0, 0, 0, 1]])

        # for i in range(self.cleaning_trajs.shape[0]):
        #     R = np.array([[1,0,0],[0,1,0],[0,0,1]])
        #     t = np.array([self.cleaning_trajs[i,0,0],self.cleaning_trajs[i,0,1],self.cleaning_trajs[i,0,2]])
        #     T = np.concatenate((np.concatenate((R,t.reshape(3,1)),axis=1),np.array([[0,0,0,1]])),axis=0)

        #     T_transformed = np.matmul(arm_z_T,T)

        #     self.cleaning_trajs[i,0,:] = T_transformed[:3,3]

        #     R = np.array([[1,0,0],[0,1,0],[0,0,1]])
        #     t = np.array([self.cleaning_trajs[i,1,0],self.cleaning_trajs[i,1,1],self.cleaning_trajs[i,1,2]])
        #     T = np.concatenate((np.concatenate((R,t.reshape(3,1)),axis=1),np.array([[0,0,0,1]])),axis=0)

        #     T_transformed = np.matmul(arm_z_T,T)

        #     self.cleaning_trajs[i,1,:] = T_transformed[:3,3]

        #hardcoded cleaning trajs and modes that are on the surface of the testbed drip pan
        # self.cleaning_trajs = np.array([[[-0.125,-0.53,-0.345],[-0.125,-0.39,-0.35]],[[0,-0.53,-0.345],[0,-0.39,-0.35]],[[0.125,-0.53,-0.345],[0.125,-0.39,-0.35]],[[-0.125,-0.39,-0.35],[-0.125,-0.05,-0.36]],[[0,-0.39,-0.35],[0,-0.05,-0.36]],[[0.125,-0.39,-0.35],[0.125,-0.05,-0.36]]])
        self.cleaning_modes = np.array([0,1]).astype(int)
        self.squeegee_width = 0.15
        self.x_min = 0.44 - self.squeegee_width / 2
        self.x_max = 0.44 + self.squeegee_width / 2
        self.y_min = -0.4
        self.y_max = 0.4
        self.complete = False

        #point clouds for visualization
        self.dirty_point_cloud = []
        self.clean_point_cloud = []

    def get_arm_z_T(self):
        T = self.tf_buffer.lookup_transform('world','robot_base', rclpy.time.Time())
        R = quaternion.as_rotation_matrix(quaternion.as_quat_array(np.array([T.transform.rotation.w,T.transform.rotation.x,T.transform.rotation.y,T.transform.rotation.z])))
        t = np.array([T.transform.translation.x, T.transform.translation.y, T.transform.translation.z]).reshape(3,1)
        arm_z_T = np.vstack((np.hstack((R,t)),np.array([0,0,0,1]).reshape(1,4)))
        return arm_z_T

    def publish_point_clouds(self):
        """
        Publishes the dirty and clean point clouds for visualization
        """
        dirty_point_cloud_msg = PointCloud()
        dirty_point_cloud_msg.header.frame_id = 'robot_base' #point cloud is in the base frame
        dirty_point_cloud_msg.points = self.dirty_point_cloud
        for i in range(len(self.dirty_point_cloud)):
            dirty_point_cloud_msg.channels.append(ChannelFloat32(name="rgb",values=[0,0,0])) #black
        dirty_point_cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.dirty_point_cloud_publisher.publish(dirty_point_cloud_msg)

        clean_point_cloud_msg = PointCloud()
        clean_point_cloud_msg.header.frame_id = 'robot_base' #point cloud is in the base frame
        clean_point_cloud_msg.points = self.clean_point_cloud
        for i in range(len(self.clean_point_cloud)):
            clean_point_cloud_msg.channels.append(ChannelFloat32(name="rgb",values=[255,255,255])) #white
        clean_point_cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.clean_point_cloud_publisher.publish(clean_point_cloud_msg)

    def point_cloud_callback(self, msg):
        """
        Callback that resets the updates the dirty point cloud and clears the clean point cloud
        """
        self.dirty_point_cloud = msg.points
        self.clean_point_cloud = []
        
        self.publish_point_clouds() #publish updated point clouds

    def update_point_clouds(self, x, y1, y2):
        """
        Updates the dirty and clean point clouds by moving points in the dirty point cloud that are within the squeegee width of the cleaning trajectory to the clean point cloud
        """
        i = 0
        y_min = min(y1,y2)
        y_max = max(y1,y2)
        while i < len(self.dirty_point_cloud):  #iterate through dirty point cloud
            point = self.dirty_point_cloud[i]
            #check if point is within squeegee width of cleaning trajectory
            if point.x >= x - self.squeegee_width / 2 and point.x <= x + self.squeegee_width / 2 and point.y >= y_min and point.y <= y_max:
                self.clean_point_cloud.append(point) #move point to clean point cloud
                self.dirty_point_cloud = self.dirty_point_cloud[:i] + self.dirty_point_cloud[i+1:] #remove point from dirty point cloud
            else:
                i = i+1
        self.publish_point_clouds() #publish updated point clouds

    def clear_clean_points(self):
        """
        Clears the clean point cloud
        """   
        self.clean_point_cloud = []
        self.publish_point_clouds() #publish updated point clouds

    def update_scene(self):
        """
        Moves the linear actuator and cart to the desired position and updates the scene
        """
        #move linear actuator and cart to desired position
        self.linear_actuator_publisher.publish(Float32(data=-0.33)) #0.55
        self.cart_publisher.publish(Float32(data=1.0)) #1.28

        self.service_complete = False
        self.future = self.scene_client.call_async(Trigger.Request())
        self.future.add_done_callback(self.move_arm_to_camera_position)
        self.get_logger().info("updating scene...")
        self.get_logger().info("scene updated")

    def move_arm_to_camera_position(self, response):
        """
        Defines a desired pose of the camera and requests the arm to move to that pose
        """
        #move are to image capture position

        arm_z_T = self.get_arm_z_T()

        R = np.array([[0,1,0],[1,0,0],[0,0,-1]])
        t = np.array([(self.x_min + self.x_max)/2, (self.y_min + self.y_max)/2, 0.18])
        T = np.concatenate((np.concatenate((R,t.reshape(3,1)),axis=1),np.array([[0,0,0,1]])),axis=0)

        T_transformed = np.matmul(arm_z_T,T)
        q_transformed = quaternion.from_rotation_matrix(T_transformed[:3,:3])

        pose_request = PoseRequest()
        pose_request.pose.position.x = T_transformed[0,3]
        pose_request.pose.position.y = T_transformed[1,3]
        pose_request.pose.position.z = T_transformed[2,3]
        pose_request.pose.orientation.x = q_transformed.x
        pose_request.pose.orientation.y = q_transformed.y
        pose_request.pose.orientation.z = q_transformed.z
        pose_request.pose.orientation.w = q_transformed.w
        pose_request.reference_frame = "left_camera"    #define the reference frame for the pose as the left camera
        self.pose_publisher.publish(pose_request) #publish the pose request

        self.get_logger().info("moving arm...")
        
    def capture_image(self):
        """
        Requests that the perception pipeline capture an image
        """
        self.get_logger().info("arm moved to image capture position")
        # get image
        self.service_complete = False
        image_request = CreosoteImage.Request()
        image_request.x_min = self.x_min
        image_request.x_max = self.x_max
        image_request.y_min = self.y_min
        image_request.y_max = self.y_max
        self.future = self.perception_client.call_async(image_request)  #call the perception pipeline to capture an image
        self.future.add_done_callback(self.segment_creosote)
        self.get_logger().info("capturing image...")
        self.get_logger().info("image captured")

    def segment_creosote(self, response):
        """
        Requests that the perception pipeline return the bounds of the creosote in an ROI
        """
        self.get_logger().info("segmenting creosote...")
        creosote_segment_request = CreosoteSegment.Request()
        x = self.cleaning_trajs[0,0,0]  #get the x position of the cleaning trajectory
        #define the ROI as the area around the cleaning trajectory
        creosote_segment_request.x_min = x - self.squeegee_width / 2
        creosote_segment_request.x_max = x + self.squeegee_width / 2
        creosote_segment_request.y_min = self.cleaning_trajs[0,0,1]
        creosote_segment_request.y_max = self.cleaning_trajs[0,1,1]
        self.future = self.segmentation_client.call_async(creosote_segment_request) #call the perception pipeline to segment the creosote
        self.future.add_done_callback(self.clean_segment)

    def clean_segment(self, response):
        """
        Requests that the arm clean the creosote from the drip pan
        """
        #get the bounds of the creosote in the ROI
        y_min = response.result().y_min_result
        y_max = response.result().y_max_result
        self.get_logger().info(f'y_min: {y_min}')
        self.get_logger().info(f'y_max: {y_max}')
        #check if creosote was found in the ROI
        if y_min < y_max:
            # self.get_logger().info(y_min)
            k = (self.cleaning_trajs[0,1,1] - y_min) / (self.cleaning_trajs[0,1,1] - self.cleaning_trajs[0,0,1])
            cleaning_request = CleaningRequest()
            #define the cleaning start and end positions and orientation

            arm_z_T = self.get_arm_z_T()

            R = np.array([[0.0, -1.0, 0.0],[0.9998477,0.0, 0.0174524],[-0.0174524, 0.0, 0.9998477]])
            t_start = np.array([self.cleaning_trajs[0,0,0], y_min, k * self.cleaning_trajs[0,0,2] + (1 - k) * self.cleaning_trajs[0,1,2]])
            t_end = np.array([self.cleaning_trajs[0,1,0], self.cleaning_trajs[0,1,1], self.cleaning_trajs[0,1,2]])

            T_start = np.concatenate((np.concatenate((R,t_start.reshape(3,1)),axis=1),np.array([[0,0,0,1]])),axis=0)
            T_end = np.concatenate((np.concatenate((R,t_end.reshape(3,1)),axis=1),np.array([[0,0,0,1]])),axis=0)
                
            T_start_transformed = np.matmul(arm_z_T,T_start)
            T_end_transformed = np.matmul(arm_z_T,T_end)
            q_transformed = quaternion.from_rotation_matrix(T_start_transformed[:3,:3])

            cleaning_request.start_position.x = T_start_transformed[0,3]
            cleaning_request.start_position.y = T_start_transformed[1,3]
            cleaning_request.start_position.z = T_start_transformed[2,3]
            cleaning_request.end_position.x = T_end_transformed[0,3]
            cleaning_request.end_position.y = T_end_transformed[1,3]
            cleaning_request.end_position.z = T_end_transformed[2,3]
            cleaning_request.orientation.x = q_transformed.x
            cleaning_request.orientation.y = q_transformed.y
            cleaning_request.orientation.z = q_transformed.z
            cleaning_request.orientation.w = q_transformed.w
            cleaning_request.mode = int(self.cleaning_modes[0]) #define the cleaning mode

            # self.get_logger().info(cleaning_request)

            self.update_point_clouds(self.cleaning_trajs[0,0,0], y_min, self.cleaning_trajs[0,1,1]) #update the point clouds

            self.clean_publisher.publish(cleaning_request) #publish the cleaning request
            self.cleaning_trajs = self.cleaning_trajs[1:,:,:]
            self.cleaning_modes = self.cleaning_modes[1:]
            self.get_logger().info("cleaning segment...")
        
        #if no creosote was found in the ROI
        else:
            self.cleaning_trajs = self.cleaning_trajs[1:,:,:] #move to the next cleaning trajectory
            self.cleaning_modes = self.cleaning_modes[1:]
            self.get_logger().info("no creosote found in segment")
            #if there are more cleaning trajectories
            if self.cleaning_trajs.shape[0] > 0:
                #segment the area of the next cleaning trajectory
                self.segment_creosote(True)
            else:
                #if there are no more cleaning trajectories, the cleaning is complete
                self.complete = True
                self.move_arm_to_camera_position(True) #move the arm to the camera position

    def move_callback(self, msg):
        """
        Callback called after camera movement is complete
        """
        if self.complete: #if cleaning is complete we're done
            self.get_logger().info("done cleaning")
        else: #if cleaning has not been completed, capture an image of the drip pan
            self.capture_image()

    def clean_callback(self, msg):
        """
        Callback called after cleaning a section is complete
        """
        self.get_logger().info("segment cleaned")
        self.clear_clean_points()   #clear the clean point cloud
        #if there are more cleaning trajectories
        if self.cleaning_trajs.shape[0] > 0:
            #segment the area of the next cleaning trajectory
            self.segment_creosote(True)
        else:
            #if there are no more cleaning trajectories, the cleaning is complete
            self.complete = True
            self.move_arm_to_camera_position(True)


def main(args=None):
    rclpy.init(args=args)

    node = MasterNode() #create the master node

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
