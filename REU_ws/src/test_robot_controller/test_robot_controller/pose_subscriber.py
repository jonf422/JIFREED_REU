#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class poseSubscriberNode(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)

    def pose_cb(self, msg: Pose):
        self.get_logger().info(str(msg))


def main(args=None):
    rclpy.init(args=args)

    node = poseSubscriberNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()