"""
Team: Carnegie Mellon MRSD Team A: CreoClean
Members: David Hill, Louis Plottel, Leonardo Mouta, Michael Gromis, Yatharth Ahuja

File: arm_scene_interface.py
Main Author: David Hill
Date: 2024-01-20

Description: ROS2 node for updating the planning scene of a UFactory 850 arm in MoveIt.
"""

import os
from ament_index_python.packages import get_package_share_directory
import rclpy
import time
import numpy as np
import quaternion
from rclpy.node import Node

from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene, AllowedCollisionEntry
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
import meshio
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class ArmSceneInterface(Node):

    def __init__(self):
        super().__init__('scene_publisher')
        # subscribe to position of linear actuator
        self.linear_actuator_position_subscriber = self.create_subscription(Float32, '/linear_actuator_position', self.linear_actuator_position_callback, 10)
        # subscribe to position of mobile platform
        self.cart_position_subscriber = self.create_subscription(Float32, '/cart_position', self.cart_position_callback, 10)
        # for publishing environment as collision objects
        self.collision_object_publisher = self.create_publisher(CollisionObject, '/collision_object', 10)
        # for publishing cleaning tool as attached collision object
        self.attached_collision_object_publisher = self.create_publisher(AttachedCollisionObject, '/attached_collision_object', 10)
        # service for updating the scene
        self.update_scene_server = self.create_service(Trigger, '/update_scene', self.update_scene_callback)
        # for updating the allowed collision matrix
        self.planning_scene_subscriber = self.create_subscription(PlanningScene, '/monitored_planning_scene', self.planning_scene_callback, 10)
        # for updating the entire planning scene at once
        self.planning_scene_publisher = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        #static transform broadcasters for end effector modes and cameras
        self.tf_broadcaster_0 = StaticTransformBroadcaster(self)
        self.tf_broadcaster_1 = StaticTransformBroadcaster(self)
        self.tf_broadcaster_2 = StaticTransformBroadcaster(self)
        self.tf_broadcaster_3 = StaticTransformBroadcaster(self)
        self.tf_broadcaster_4 = StaticTransformBroadcaster(self)

        # default parameters can be changed in the launch file
        self.declare_parameters(
            namespace='',
            parameters=[
                ('cad_namespace', 'testbed'),
                ('stl_scale', 0.001),
                ('stl_load_range', 3.0),
                ('position_offset', [0.0,0.0,0.0]),
                ('orientation_offset', [0.0,0.0,0.0,1.0]),
                ('section_widths', [2.0]),
                ('section_displacements', [0.0])
            ])

        #save parameters to class variables
        self.stl_directory = os.path.join(get_package_share_directory('drip_pan_environment'),'meshes')
        self.cad_namespace = self.get_parameter('cad_namespace').get_parameter_value()._string_value
        self.stl_scale = self.get_parameter('stl_scale').get_parameter_value()._double_value
        self.stl_load_range = self.get_parameter('stl_load_range').get_parameter_value()._double_value
        self.position_offset = self.get_parameter('position_offset').get_parameter_value()._double_array_value
        self.orientation_offset = self.get_parameter('orientation_offset').get_parameter_value()._double_array_value
        self.section_widths = self.get_parameter('section_widths').get_parameter_value()._double_array_value
        self.section_displacements = self.get_parameter('section_displacements').get_parameter_value()._double_array_value
        self.active_sections = [False for i in range(len(self.section_widths))]
        # self.get_logger().info(f'{self.orientation_offset}', once=True)
        
        #initialize variables
        self.linear_actuator_position = 0.0
        self.cart_position = 0.0
        self.collision_matrix_needs_update = False

        #initialize planning scene
        self.publish_ee_transforms()

        tf_future = self.tf_buffer.wait_for_transform_async(
                    target_frame='world',
                    source_frame='robot_base',
                    time=rclpy.time.Time()
                )

        rclpy.spin_until_future_complete(self, tf_future)

        tf_future = self.tf_buffer.wait_for_transform_async(
                    target_frame='world',
                    source_frame='link_eef',
                    time=rclpy.time.Time()
                )

        rclpy.spin_until_future_complete(self, tf_future)

        self.publish_scene_objects(update_tool=True)

    def planning_scene_callback(self, msg):
        """
        Callback function for the planning scene subscriber. This function updates the allowed collision matrix.
        """
        if self.collision_matrix_needs_update:
            self.update_allowed_collision_matrix(msg)
            self.collision_matrix_needs_update = False

    def update_allowed_collision_matrix(self, msg):
        """
        Updates the allowed collision matrix of the planning scene. This function is called by the planning_scene_callback function.
        """
        self.latest_planning_scene = msg
        self.latest_planning_scene.allowed_collision_matrix.entry_names = ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link_base', 'link_eef', 'cleaning_tool', 'squeegee', 'walls', 'robot_chassis', self.cad_namespace + '0']
        self.latest_planning_scene.allowed_collision_matrix.entry_values = []                                 #link1, link2, link3, link4, link5, link6, base, eef, tool, squee, walls, chass, pan
        self.latest_planning_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[False, True, True, False, False, False, True, False, False, False, True, False, False]))    #link1
        self.latest_planning_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[True, False, True, True, False, False, True, False, False, False, True, False, False]))     #link2
        self.latest_planning_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[True, True, False, True, True, True, False, False, False, False, True, False, False]))      #link3
        self.latest_planning_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[False, True, True, False, True, True, False, False, False, False, True, False, False]))     #link4
        self.latest_planning_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[False, False, True, True, False, True, False, False, False, False, True, False, False]))    #link5
        self.latest_planning_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[False, False, True, True, True, False, False, True, True, False, True, False, False]))      #link6
        self.latest_planning_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[True, True, False, False, False, False, False, False, False, False, True, False, False]))   #link_base
        self.latest_planning_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[False, False, False, False, False, True, False, False, True, False, True, False, False]))   #link_eef
        self.latest_planning_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[False, False, False, False, False, True, False, True, False, True, True, False, False]))  #cleaning_tool
        self.latest_planning_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[False, False, False, False, False, False, False, False, True, False, False, False, True]))  #squeegee
        self.latest_planning_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[True, True, True, True, True, True, True, True, True, False, False, True, True]))           #walls
        self.latest_planning_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[False, False, False, False, False, False, False, False, False, False, True, False, True]))  #robot_chassis
        self.latest_planning_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[False, False, False, False, False, False, False, False, False, True, True, True, False]))  #drip pan
        self.planning_scene_publisher.publish(self.latest_planning_scene)
        self.get_logger().info('allowed collision matrix updated')
    
    def publish_ee_transforms(self):
        """
        Publishes the transforms of the end effector modes and cameras.
        """
        # end effector mode 0 (pushing mode) found experimentally
        msg = TransformStamped()
        msg.transform.translation.x = 0.12
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = 0.235
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.9914449
        msg.transform.rotation.z = 0.0
        msg.transform.rotation.w = 0.1305262

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_eef'
        msg.child_frame_id = 'ee_mode_0'

        self.tf_broadcaster_0.sendTransform(msg)

        # end effector mode 1 (pulling mode) found experimentally
        msg = TransformStamped()
        msg.transform.translation.x = 0.12
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = 0.235
        msg.transform.rotation.x = 0.9914449
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = 0.1305262
        msg.transform.rotation.w = 0.0

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_eef'
        msg.child_frame_id = 'ee_mode_1'

        self.tf_broadcaster_1.sendTransform(msg)

        # left camera found experimentally
        msg = TransformStamped()
        msg.transform.translation.x = -0.05
        msg.transform.translation.y = 0.06
        msg.transform.translation.z = 0.037
        msg.transform.rotation.x = 0.5
        msg.transform.rotation.y = -0.5
        msg.transform.rotation.z = -0.5
        msg.transform.rotation.w = 0.5

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_eef'
        msg.child_frame_id = 'left_camera'

        self.tf_broadcaster_2.sendTransform(msg)

        # right camera found experimentally
        msg = TransformStamped()
        msg.transform.translation.x = -0.06
        msg.transform.translation.y = -0.05
        msg.transform.translation.z = 0.037
        msg.transform.rotation.x = 0.5
        msg.transform.rotation.y = -0.5
        msg.transform.rotation.z = -0.5
        msg.transform.rotation.w = 0.5

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_eef'
        msg.child_frame_id = 'right_camera'

        self.tf_broadcaster_3.sendTransform(msg)

        # robot base to world transform
        msg = TransformStamped()
        msg.transform.translation.x = 0.0
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = 0.0
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = -0.9238795
        msg.transform.rotation.w = 0.3826834

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'robot_base'

        self.tf_broadcaster_4.sendTransform(msg)


    def cart_position_callback(self, msg):
        """
        Callback function for the cart position subscriber. This function updates the cart position variable.
        """
        self.cart_position = msg.data

    def linear_actuator_position_callback(self, msg):
        """
        Callback function for the linear actuator position subscriber. This function updates the linear actuator position variable.
        """
        self.linear_actuator_position = msg.data

    def publish_scene_objects(self, update_tool=False):
        """
        Function for publishing the scene objects. This function adds the cleaning tool, walls, mobile platform, and sections of the drip pan to the scene.
        """

        T = self.tf_buffer.lookup_transform('world','robot_base', rclpy.time.Time())
        R = quaternion.as_rotation_matrix(quaternion.as_quat_array(np.array([T.transform.rotation.w,T.transform.rotation.x,T.transform.rotation.y,T.transform.rotation.z])))
        t = np.array([T.transform.translation.x, T.transform.translation.y, T.transform.translation.z]).reshape(3,1)
        arm_z_T = np.vstack((np.hstack((R,t)),np.array([0,0,0,1]).reshape(1,4)))

        # arm_z_rotation = -2.35619449019 #0.78539816339
        # arm_z_T = np.array([[np.cos(arm_z_rotation), -np.sin(arm_z_rotation), 0, 0.0],
        #               [np.sin(arm_z_rotation), np.cos(arm_z_rotation), 0, 0.0],
        #               [0, 0, 1, 0.0],
        #               [0, 0, 0, 1]])

        if update_tool: #if the has not been added to the scene yet

            T = self.tf_buffer.lookup_transform('world','link_eef', rclpy.time.Time())
            R = quaternion.as_rotation_matrix(quaternion.as_quat_array(np.array([T.transform.rotation.w,T.transform.rotation.x,T.transform.rotation.y,T.transform.rotation.z])))
            t = np.array([T.transform.translation.x, T.transform.translation.y, T.transform.translation.z]).reshape(3,1)
            T_mat = np.vstack((np.hstack((R,t)),np.array([0,0,0,1]).reshape(1,4)))

            #add cleaning tool
            msg = AttachedCollisionObject()
            msg.link_name = "link6" #cleaning tool is attached to the last link of the arm
            
            R = np.array([[0,0,-1],[-1,0,0],[0,1,0]])
            t = np.array([0.14,0.084,-0.055])
            T = np.concatenate((np.concatenate((R,t.reshape(3,1)),axis=1),np.array([[0,0,0,1]])),axis=0)

            T_transformed = np.matmul(T_mat,T)
            q_transformed = quaternion.from_rotation_matrix(T_transformed[0:3,0:3])

            msg.object.pose.position.x = T_transformed[0,3]
            msg.object.pose.position.y = T_transformed[1,3]
            msg.object.pose.position.z = T_transformed[2,3]
            msg.object.pose.orientation.x = q_transformed.x
            msg.object.pose.orientation.y = q_transformed.y
            msg.object.pose.orientation.z = q_transformed.z
            msg.object.pose.orientation.w = q_transformed.w
            
            # msg.object.pose.position.x = 0.238
            # msg.object.pose.position.y = 0.138
            # msg.object.pose.position.z = 0.30
            # msg.object.pose.orientation.x = 0.0
            # msg.object.pose.orientation.y = -0.7071068
            # msg.object.pose.orientation.z = 0.7071068
            # msg.object.pose.orientation.w = 0.0
            msg.object.id = "cleaning_tool"
            msg.touch_links = ["link6"]

            #load stl file of cleaning tool
            raw_mesh = meshio.read(os.path.join(self.stl_directory,'cleaning_tool.STL'))
            moveit_mesh = Mesh()

            #format stl file for moveit
            for p in raw_mesh.points:
                moveit_mesh.vertices.append(Point(x=p[0]*self.stl_scale,y=p[1]*self.stl_scale,z=p[2]*self.stl_scale))
            for v in raw_mesh.cells_dict['triangle']:
                moveit_mesh.triangles.append(MeshTriangle(vertex_indices=v))
            msg.object.meshes = [moveit_mesh]
            msg.object.operation = bytes([0])

            msg.object.header.stamp = self.get_clock().now().to_msg()

            # planning_scene_msg.robot_state.attached_collision_objects.append(msg)
            # planning_scene_msg.allowed_collision_matrix.entry_names.append('cleaning_tool')
            # planning_scene_msg.allowed_collision_matrix.entry_values.append()

            self.attached_collision_object_publisher.publish(msg)

            #add squeegee (only squeegee will be constrained to be above the pan)
            msg = AttachedCollisionObject()
            msg.link_name = "link6" #squeegee is attached to the last link of the arm
            
            R = np.array([[0,0,-1],[-1,0,0],[0,1,0]])
            t = np.array([0.14,0.074,-0.055])
            T = np.concatenate((np.concatenate((R,t.reshape(3,1)),axis=1),np.array([[0,0,0,1]])),axis=0)

            T_transformed = np.matmul(T_mat,T)
            q_transformed = quaternion.from_rotation_matrix(T_transformed[0:3,0:3])
            
            msg.object.pose.position.x = T_transformed[0,3]
            msg.object.pose.position.y = T_transformed[1,3]
            msg.object.pose.position.z = T_transformed[2,3]
            msg.object.pose.orientation.x = q_transformed.x
            msg.object.pose.orientation.y = q_transformed.y
            msg.object.pose.orientation.z = q_transformed.z
            msg.object.pose.orientation.w = q_transformed.w
            msg.object.id = "squeegee"
            msg.touch_links = ["link6"]

            #load stl file of squeegee
            raw_mesh = meshio.read(os.path.join(self.stl_directory,'squeegee.STL'))
            moveit_mesh = Mesh()

            #format stl file for moveit
            for p in raw_mesh.points:
                moveit_mesh.vertices.append(Point(x=p[0]*self.stl_scale,y=p[1]*self.stl_scale,z=p[2]*self.stl_scale))
            for v in raw_mesh.cells_dict['triangle']:
                moveit_mesh.triangles.append(MeshTriangle(vertex_indices=v))
            msg.object.meshes = [moveit_mesh]
            msg.object.operation = bytes([0])

            msg.object.header.stamp = self.get_clock().now().to_msg()

            self.attached_collision_object_publisher.publish(msg)

        #add walls
        msg = CollisionObject()
        msg.header.frame_id = "world"
        msg.id = 'walls'

        R1 = np.array([[1,0,0],[0,1,0],[0,0,1]])
        t1 = np.array([0.915 - self.cart_position,-1.11 + self.linear_actuator_position,0.5])
        T1 = np.concatenate((np.concatenate((R1,t1.reshape(3,1)),axis=1),np.array([[0,0,0,1]])),axis=0)

        T1_transformed = np.matmul(arm_z_T,T1)
        q1_transformed = quaternion.from_rotation_matrix(T1_transformed[0:3,0:3])

        #first wall
        wall_1 = SolidPrimitive()
        wall_1.type = SolidPrimitive.BOX
        wall_1.dimensions = [1.83,0.01,2.0]
        wall_1_pose = Pose()
        wall_1_pose.position.x = T1_transformed[0,3]
        wall_1_pose.position.y = T1_transformed[1,3]
        wall_1_pose.position.z = T1_transformed[2,3]
        wall_1_pose.orientation.x = q1_transformed.x
        wall_1_pose.orientation.y = q1_transformed.y
        wall_1_pose.orientation.z = q1_transformed.z
        wall_1_pose.orientation.w = q1_transformed.w

        msg.primitives.append(wall_1)
        msg.primitive_poses.append(wall_1_pose)

        R2 = np.array([[1,0,0],[0,1,0],[0,0,1]])
        t2 = np.array([1.84 - self.cart_position, -0.12 + self.linear_actuator_position,0.5])
        T2 = np.concatenate((np.concatenate((R2,t2.reshape(3,1)),axis=1),np.array([[0,0,0,1]])),axis=0)

        T2_transformed = np.matmul(arm_z_T,T2)
        q2_transformed = quaternion.from_rotation_matrix(T2_transformed[0:3,0:3])

        #second wall
        wall_2 = SolidPrimitive()
        wall_2.type = SolidPrimitive.BOX
        wall_2.dimensions = [0.01,2.0,2.0]
        wall_2_pose = Pose()
        wall_2_pose.position.x = T2_transformed[0,3]
        wall_2_pose.position.y = T2_transformed[1,3]
        wall_2_pose.position.z = T2_transformed[2,3]
        wall_2_pose.orientation.x = q2_transformed.x
        wall_2_pose.orientation.y = q2_transformed.y
        wall_2_pose.orientation.z = q2_transformed.z
        wall_2_pose.orientation.w = q2_transformed.w

        msg.primitives.append(wall_2)
        msg.primitive_poses.append(wall_2_pose)

        R3 = np.array([[1,0,0],[0,1,0],[0,0,1]])
        t3 = np.array([-0.01 - self.cart_position, -0.12 + self.linear_actuator_position,0.5])
        T3 = np.concatenate((np.concatenate((R3,t3.reshape(3,1)),axis=1),np.array([[0,0,0,1]])),axis=0)

        T3_transformed = np.matmul(arm_z_T,T3)
        q3_transformed = quaternion.from_rotation_matrix(T3_transformed[0:3,0:3])

        #third wall
        wall_3 = SolidPrimitive()
        wall_3.type = SolidPrimitive.BOX
        wall_3.dimensions = [0.01,2.0,2.0]
        wall_3_pose = Pose()
        wall_3_pose.position.x = T3_transformed[0,3]
        wall_3_pose.position.y = T3_transformed[1,3]
        wall_3_pose.position.z = T3_transformed[2,3]
        wall_3_pose.orientation.x = q3_transformed.x
        wall_3_pose.orientation.y = q3_transformed.y
        wall_3_pose.orientation.z = q3_transformed.z
        wall_3_pose.orientation.w = q3_transformed.w

        msg.primitives.append(wall_3)
        msg.primitive_poses.append(wall_3_pose)

        msg.header.stamp = self.get_clock().now().to_msg()

        # planning_scene_msg.world.collision_objects.append(msg)

        self.collision_object_publisher.publish(msg)

        #add mobile platform
        msg = CollisionObject()
        msg.header.frame_id = "world"
        msg.pose = Pose()

        R_mobile_platform = np.array([[0,0,-1],[-1,0,0],[0,1,0]])
        t_mobile_platform = np.array([0.254,0.76 + self.linear_actuator_position,-0.19])
        T_mobile_platform = np.concatenate((np.concatenate((R_mobile_platform,t_mobile_platform.reshape(3,1)),axis=1),np.array([[0,0,0,1]])),axis=0)

        T_mobile_platform_transformed = np.matmul(arm_z_T,T_mobile_platform)
        q_mobile_platform_transformed = quaternion.from_rotation_matrix(T_mobile_platform_transformed[0:3,0:3])

        msg.pose.position.x = T_mobile_platform_transformed[0,3]
        msg.pose.position.y = T_mobile_platform_transformed[1,3]
        msg.pose.position.z = T_mobile_platform_transformed[2,3]
        msg.pose.orientation.x = q_mobile_platform_transformed.x
        msg.pose.orientation.y = q_mobile_platform_transformed.y
        msg.pose.orientation.z = q_mobile_platform_transformed.z
        msg.pose.orientation.w = q_mobile_platform_transformed.w
        msg.id = 'robot_chassis'

        #load stl file of mobile platform
        raw_mesh = meshio.read(os.path.join(self.stl_directory,'robot_chassis.STL'))
        moveit_mesh = Mesh()

        #format stl file for moveit
        for p in raw_mesh.points:

            moveit_mesh.vertices.append(Point(x=p[0]*self.stl_scale,y=p[1]*self.stl_scale,z=p[2]*self.stl_scale))
        for v in raw_mesh.cells_dict['triangle']:
            moveit_mesh.triangles.append(MeshTriangle(vertex_indices=v))
        msg.meshes = [moveit_mesh]
        msg.operation = bytes([0])

        msg.header.stamp = self.get_clock().now().to_msg()

        self.collision_object_publisher.publish(msg)

        #add desired environment
        for i in range(len(self.section_widths)):   #for each section of the environment
            #check if the section is near the robot
            min_cart_position = self.section_displacements[i] - self.section_widths[i]/2 - self.stl_load_range
            max_cart_position = self.section_displacements[i] + self.section_widths[i]/2 + self.stl_load_range
            #only add the section to the scene if it is near the robot
            if(self.cart_position >= min_cart_position and self.cart_position <= max_cart_position):
                msg = CollisionObject()
                msg.header.frame_id = "world"
                msg.pose = Pose()

                q = np.quaternion(self.orientation_offset[3],self.orientation_offset[0],self.orientation_offset[1],self.orientation_offset[2])
                R = quaternion.as_rotation_matrix(q)
                t = np.array([self.position_offset[0] - self.cart_position, self.position_offset[1] + self.linear_actuator_position, self.position_offset[2]])
                T = np.concatenate((np.concatenate((R,t.reshape(3,1)),axis=1),np.array([[0,0,0,1]])),axis=0)

                T_transformed = np.matmul(arm_z_T,T)
                q_transformed = quaternion.from_rotation_matrix(T_transformed[0:3,0:3])

                msg.pose.position.x = T_transformed[0,3]
                msg.pose.position.y = T_transformed[1,3]
                msg.pose.position.z = T_transformed[2,3]
                msg.pose.orientation.x = q_transformed.x
                msg.pose.orientation.y = q_transformed.y
                msg.pose.orientation.z = q_transformed.z
                msg.pose.orientation.w = q_transformed.w
                msg.id = self.cad_namespace + str(i)    #make the id of the object based on the section number

                #load stl file of section
                raw_mesh = meshio.read(os.path.join(self.stl_directory,self.cad_namespace + str(i)+'.STL'))
                moveit_mesh = Mesh()

                #format stl file for moveit
                for p in raw_mesh.points:
                    moveit_mesh.vertices.append(Point(x=p[0]*self.stl_scale,y=p[1]*self.stl_scale,z=p[2]*self.stl_scale))
                for v in raw_mesh.cells_dict['triangle']:
                    moveit_mesh.triangles.append(MeshTriangle(vertex_indices=v))
                msg.meshes = [moveit_mesh]
                msg.operation = bytes([0])

                msg.header.stamp = self.get_clock().now().to_msg()

                self.collision_object_publisher.publish(msg)

                self.active_sections[i] = True
            
            #if the section is not near the robot and was previously in the scene, remove it
            elif(self.active_sections[i]):
                msg = CollisionObject()
                msg.header.frame_id = "world"
                msg.id = self.cad_namespace + str(i)
                msg.operation = bytes([1])

                msg.header.stamp = self.get_clock().now().to_msg()

                self.collision_object_publisher.publish(msg)

        self.collision_matrix_needs_update = True   #set flag to update the allowed collision matrix
        return True

    def update_scene_callback(self, request, response):
        """
        Callback function for the update scene service. This function updates the scene.
        """
        response.success = self.publish_scene_objects()
        return response

def main(args=None):
    rclpy.init(args=args) #initialize ROS2 node

    scene_interface = ArmSceneInterface()   #initialize the scene interface node

    rclpy.spin(scene_interface)

    scene_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
