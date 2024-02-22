import rclpy
import time
import numpy as np
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point, Quaternion
import meshio


class ScenePublisher(Node):

    def __init__(self):
        super().__init__('scene_publisher')
        self.publisher_ = self.create_publisher(CollisionObject, '/collision_object', 10)
        timer_period = 0.01 # seconds
        self.timer = self.create_timer(timer_period, self.publish)
        self.i = 0
        self.publish()

    def publish(self):
        if self.i == 0:
            msg = CollisionObject()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.pose = Pose()
            msg.pose.position.x = -1.0
            msg.pose.position.y = 1.0
            msg.pose.position.z = -0.3
            msg.pose.orientation.x = 1.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            print(msg.pose)
            msg.id = "custom_1"

            raw_mesh = meshio.read('/home/david-hill/dev_ws/src/koppers_ros2/meshes/testbed.STL')
            moveit_mesh = Mesh()
            # print(numpy_mesh.points.shape)
            scale = 0.001
            for p in raw_mesh.points:
                moveit_mesh.vertices.append(Point(x=p[0]*scale,y=p[1]*scale,z=p[2]*scale))
            for v in raw_mesh.cells_dict['triangle']:
                moveit_mesh.triangles.append(MeshTriangle(vertex_indices=v))
            msg.meshes = [moveit_mesh]

            # msg.pose.position.x = self.i/2.5 - 1.0
            # msg.pose.position.y = (self.i/5)/2.5 - 1.0
            # msg.pose.position.z = (self.i/25)/2.5 - 1.0
            # box_1 = SolidPrimitive()
            # box_1.type = 1
            # box_1.dimensions = [0.1,0.1,0.1]
            # msg.primitives = [box_1]

            msg.operation = bytes([0])
            self.publisher_.publish(msg)
            self.get_logger().info('Published: message')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    publisher = ScenePublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
