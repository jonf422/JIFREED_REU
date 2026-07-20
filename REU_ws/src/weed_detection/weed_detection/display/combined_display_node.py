#!/usr/bin/env python3
'''
combined_display_node.py

Subscribes to all three camera feeds and displays them side-by-side in one window.

Topics
------
/vision/arducam_display     (sensor_msgs/Image, bgr8)
/vision/realsense_display   (sensor_msgs/Image, bgr8)
/vision/realsense_depth     (sensor_msgs/Image, 16UC1)

Usage
-----
ros2 run weed_detection combined_display_node
'''
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

PANEL_HEIGHT = 480
PLACEHOLDER_WIDTH = 640


class CombinedDisplayNode(Node):
    def __init__(self):
        super().__init__("combined_display_node")

        self.declare_parameter('depth_fps', 30)
        self._depth_fps = self.get_parameter('depth_fps').value

        self._bridge = CvBridge()
        self._arducam_frame = None
        self._realsense_frame = None
        self._depth_frame = None
        self._log_counter = 0

        self.create_subscription(Image, "/vision/arducam_display",   self._arducam_cb,   SENSOR_QOS)
        self.create_subscription(Image, "/vision/realsense_color", self._realsense_cb, SENSOR_QOS)
        self.create_subscription(Image, "/vision/realsense_depth",   self._depth_cb,     SENSOR_QOS)

        self.get_logger().info("combined_display_node started")

    def _arducam_cb(self, msg: Image) -> None:
        self._arducam_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._refresh()

    def _realsense_cb(self, msg: Image) -> None:
        self._realsense_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._refresh()

    def _depth_cb(self, msg: Image) -> None:
        depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        h, w = depth.shape
        center_mm = int(depth[h // 2, w // 2])

        depth_clipped = np.clip(depth, 0, 5000)
        depth_normalized = (depth_clipped / 5000 * 255).astype(np.uint8)
        self._depth_frame = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

        self._log_counter += 1
        if self._log_counter % self._depth_fps == 0:
            self.get_logger().info(f"Center depth: {center_mm}mm ({center_mm / 1000:.2f}m)")

        self._refresh()

    def _make_panel(self, frame, label: str) -> np.ndarray:
        if frame is None:
            panel = np.zeros((PANEL_HEIGHT, PLACEHOLDER_WIDTH, 3), dtype=np.uint8)
            cv2.putText(panel, label,     (10, PANEL_HEIGHT // 2 - 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (160, 160, 160), 1, cv2.LINE_AA)
            cv2.putText(panel, 'waiting', (10, PANEL_HEIGHT // 2 + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (160, 160, 160), 1, cv2.LINE_AA)
            return panel

        h, w = frame.shape[:2]
        new_w = max(1, int(w * PANEL_HEIGHT / h))
        panel = cv2.resize(frame, (new_w, PANEL_HEIGHT))

        # label with black outline for readability on any background
        cv2.putText(panel, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,   0,   0  ), 3, cv2.LINE_AA)
        cv2.putText(panel, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1, cv2.LINE_AA)
        return panel

    def _refresh(self) -> None:
        panels = [
            self._make_panel(self._arducam_frame,   'Arducam'),
            self._make_panel(self._realsense_frame, 'RealSense'),
            self._make_panel(self._depth_frame,     'Depth'),
        ]
        cv2.imshow('Camera Feeds', np.hstack(panels))
        cv2.waitKey(1)

    def destroy_node(self) -> None:
        self.get_logger().info("Destroying combined_display_node")
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CombinedDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()