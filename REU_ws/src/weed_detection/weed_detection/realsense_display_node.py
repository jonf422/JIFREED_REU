#!/usr/bin/env python3
'''
realsense_display_node.py

Subscribes to /vision/realsense_display

Usage
-----
ros2 run weed_detection display_node
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)

class realsenseDisplayNode(Node):
    def __init__(self):
        super().__init__("realsense_display_node")

        self._bridge = CvBridge()
        self._realsense_color_sub = self.create_subscription(Image, "/vision/realsense_display", self.realsense_display_cb, SENSOR_QOS)

        self.get_logger().info("realsense_display_node Started")

    def realsense_display_cb(self, msg:Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('RealSense', frame)
        cv2.waitKey(1)

    def destroy_node(self) -> None:
        self.get_logger().info("Destroying realsense_display_node")
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = realsenseDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
