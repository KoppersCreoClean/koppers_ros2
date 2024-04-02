import os
from ament_index_python.packages import get_package_share_directory
import rclpy
import time
import numpy as np
import quaternion
from rclpy.node import Node

from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
import meshio
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class ArmSceneInterface(Node):

    def __init__(self):
        super().__init__('scene_publisher')
        self.linear_actuator_position_subscriber = self.create_subscription(Float32, '/linear_actuator_position', self.linear_actuator_position_callback, 10)
        self.cart_position_subscriber = self.create_subscription(Float32, '/cart_position', self.cart_position_callback, 10)
        self.collision_object_publisher = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.attached_collision_object_publisher = self.create_publisher(AttachedCollisionObject, '/attached_collision_object', 10)
        self.update_scene_server = self.create_service(Trigger, '/update_scene', self.update_scene_callback)

        self.tf_broadcaster_0 = StaticTransformBroadcaster(self)
        self.tf_broadcaster_1 = StaticTransformBroadcaster(self)

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
        
        self.linear_actuator_position = 0.0
        self.cart_position = 0.0

        self.publish_ee_transforms()
        self.publish_scene_objects(update_tool=True)

    def publish_ee_transforms(self):
        # end effector mode 0
        msg = TransformStamped()
        msg.transform.translation.x = 0.125
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = 0.24
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.9914449
        msg.transform.rotation.z = 0.0
        msg.transform.rotation.w = 0.1305262

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link6'
        msg.child_frame_id = 'ee_mode_0'

        self.tf_broadcaster_0.sendTransform(msg)

        # end effector mode 1
        msg = TransformStamped()
        msg.transform.translation.x = 0.125
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = 0.24
        msg.transform.rotation.x = 0.9914449
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = 0.1305262
        msg.transform.rotation.w = 0.0

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link6'
        msg.child_frame_id = 'ee_mode_1'

        self.tf_broadcaster_1.sendTransform(msg)


    def cart_position_callback(self, msg):
        self.cart_position = msg.data

    def linear_actuator_position_callback(self, msg):
        self.linear_actuator_position = msg.data

    def publish_scene_objects(self, update_tool=False):
        
        if update_tool:
            #add cleaning tool
            msg = AttachedCollisionObject()
            msg.link_name = "link6"
            msg.object.pose.position.x = 0.29
            msg.object.pose.position.y = -0.075
            msg.object.pose.position.z = 0.298
            msg.object.pose.orientation.x = -0.5
            msg.object.pose.orientation.y = -0.5
            msg.object.pose.orientation.z = 0.5
            msg.object.pose.orientation.w = 0.5
            msg.object.id = "cleaning_tool"
            msg.touch_links = ["link6"]

            raw_mesh = meshio.read(os.path.join(self.stl_directory,'cleaning_tool.STL'))
            moveit_mesh = Mesh()

            for p in raw_mesh.points:
                moveit_mesh.vertices.append(Point(x=p[0]*self.stl_scale,y=p[1]*self.stl_scale,z=p[2]*self.stl_scale))
            for v in raw_mesh.cells_dict['triangle']:
                moveit_mesh.triangles.append(MeshTriangle(vertex_indices=v))
            msg.object.meshes = [moveit_mesh]
            msg.object.operation = bytes([0])

            msg.object.header.stamp = self.get_clock().now().to_msg()

            self.attached_collision_object_publisher.publish(msg)

        #add mobile platform
        msg = CollisionObject()
        msg.header.frame_id = "world"
        msg.pose = Pose()
        msg.pose.position.x = 0.254
        msg.pose.position.y = 0.76 + self.linear_actuator_position
        msg.pose.position.z = -0.19
        msg.pose.orientation.x = -0.5
        msg.pose.orientation.y = 0.5
        msg.pose.orientation.z = 0.5
        msg.pose.orientation.w = -0.5
        msg.id = 'robot_chassis'

        raw_mesh = meshio.read(os.path.join(self.stl_directory,'robot_chassis.STL'))
        moveit_mesh = Mesh()

        for p in raw_mesh.points:
            moveit_mesh.vertices.append(Point(x=p[0]*self.stl_scale,y=p[1]*self.stl_scale,z=p[2]*self.stl_scale))
        for v in raw_mesh.cells_dict['triangle']:
            moveit_mesh.triangles.append(MeshTriangle(vertex_indices=v))
        msg.meshes = [moveit_mesh]
        msg.operation = bytes([0])

        msg.header.stamp = self.get_clock().now().to_msg()

        self.collision_object_publisher.publish(msg)

        #add desired environment
        for i in range(len(self.section_widths)):
            min_cart_position = self.section_displacements[i] - self.section_widths[i]/2 - self.stl_load_range
            max_cart_position = self.section_displacements[i] + self.section_widths[i]/2 + self.stl_load_range
            if(self.cart_position >= min_cart_position and self.cart_position <= max_cart_position):
                msg = CollisionObject()
                msg.header.frame_id = "world"
                msg.pose = Pose()
                msg.pose.position.x = self.position_offset[0] - self.cart_position
                msg.pose.position.y = self.position_offset[1] + self.linear_actuator_position
                msg.pose.position.z = self.position_offset[2]
                msg.pose.orientation.x = self.orientation_offset[0]
                msg.pose.orientation.y = self.orientation_offset[1]
                msg.pose.orientation.z = self.orientation_offset[2]
                msg.pose.orientation.w = self.orientation_offset[3]
                msg.id = self.cad_namespace + str(i)

                raw_mesh = meshio.read(os.path.join(self.stl_directory,self.cad_namespace + str(i)+'.STL'))
                moveit_mesh = Mesh()

                for p in raw_mesh.points:
                    moveit_mesh.vertices.append(Point(x=p[0]*self.stl_scale,y=p[1]*self.stl_scale,z=p[2]*self.stl_scale))
                for v in raw_mesh.cells_dict['triangle']:
                    moveit_mesh.triangles.append(MeshTriangle(vertex_indices=v))
                msg.meshes = [moveit_mesh]
                msg.operation = bytes([0])

                msg.header.stamp = self.get_clock().now().to_msg()

                self.collision_object_publisher.publish(msg)

                self.active_sections[i] = True
            
            elif(self.active_sections[i]):
                msg = CollisionObject()
                msg.header.frame_id = "world"
                msg.id = self.cad_namespace + str(i)
                msg.operation = bytes([1])

                msg.header.stamp = self.get_clock().now().to_msg()

                self.collision_object_publisher.publish(msg)
        return True

    def update_scene_callback(self, request, response):
        response.success = self.publish_scene_objects()
        return response

def main(args=None):
    rclpy.init(args=args)

    scene_interface = ArmSceneInterface()

    rclpy.spin(scene_interface)

    scene_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
