#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)

class depthDisplayNode(Node):
    def __init__(self):
        super().__init__("realsense_depth_display_node")

        #ROS2 parameters
        self.declare_parameter('width',  640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps',     30)

        self.w = self.get_parameter('width').value
        self.h = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value

        self._bridge = CvBridge()
        self._depth_sub = self.create_subscription(Image, "/vision/realsense_depth", self.depth_display_cb, SENSOR_QOS)

        #center depth ctr log 1/sec
        self._log_counter = 0

        self.get_logger().info("realsense_depth_display_node Started")

    def depth_display_cb(self, msg: Image) -> None:
        depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        
        # Normalize to 0-255 for display (clips at 5 metres)
        depth_clipped = np.clip(depth, 0, 5000)
        depth_normalized = (depth_clipped / 5000 * 255).astype(np.uint8)
        
        # Apply colormap so depth is easier to read visually
        depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
        
        cv2.imshow('Depth', depth_colored)
        cv2.moveWindow('Depth', 660, 0)
        cv2.waitKey(1)

        
        center_depth = depth[self.h//2, self.w//2]

        self._log_counter += 1
        #log once every second
        if self._log_counter % self.fps == 0: 
            self.get_logger().info(f"Center pixel depth: {center_depth}mm ({center_depth/1000:.2f}m)")

    def destroy_node(self) -> None:
        self.get_logger().info("Destroying realsense_display_node")
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = depthDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

