#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class testNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self.get_logger().info("ROS2 Test Node")

        self.counter_ = 0
        self.create_timer(1.0, self.timer_cb)
    
    def timer_cb(self):
        self.get_logger().info("Timer " + str(self.counter_))
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)

    node = testNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()