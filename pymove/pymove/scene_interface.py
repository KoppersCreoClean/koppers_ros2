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
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point, Quaternion


class ScenePublisher(Node):

    def __init__(self):
        super().__init__('scene_publisher')
        self.publisher_ = self.create_publisher(CollisionObject, '/collision_object', 10)
        # timer_period = 1 # seconds
        # self.timer = self.create_timer(timer_period, self.publish)
        self.i = 0
        self.publish()

    def publish(self):
        msg = CollisionObject()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.id = "box_1"
        box_1 = SolidPrimitive()
        box_1.type = 1
        box_1.dimensions = [0.5,0.5,0.5]
        msg.primitives = [box_1]
        box_1_pose = Pose()
        box_1_position = Point()
        box_1_position.x = 1.0
        box_1_position.y = 1.0
        box_1_position.z = 0.0
        box_1_pose.position = box_1_position
        msg.primitive_poses = [box_1_pose]
        msg.operation = bytes([0])
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "%s":' % msg)
        # self.i += 1

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
