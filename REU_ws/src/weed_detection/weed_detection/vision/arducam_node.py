"""
arducam_node.py

Sole owner of the Arducam V4L2 device.  Publishes raw frames so that any
node needing the Arducam feed subscribes here instead of opening the device
directly.

Topics published
----------------
/vision/arducam_raw      (sensor_msgs/Image, bgr8)  — processing feed
/vision/arducam_display  (sensor_msgs/Image, bgr8)  — display feed

Parameters
----------
device_id   int   0      V4L2 device index (cv2.VideoCapture argument)
width       int   640
height      int   480
fps         int   30
"""
from __future__ import annotations

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


class ArducamNode(Node):

    def __init__(self):
        super().__init__('arducam_node')

        self.declare_parameter('device_id', 0)
        self.declare_parameter('width',   640)
        self.declare_parameter('height',  480)
        self.declare_parameter('fps',      30)

        device_id = self.get_parameter('device_id').value
        width     = self.get_parameter('width').value
        height    = self.get_parameter('height').value
        fps       = self.get_parameter('fps').value

        self._bridge = CvBridge()

        self._raw_pub     = self.create_publisher(
            Image, '/vision/arducam_raw',     SENSOR_QOS)
        self._display_pub = self.create_publisher(
            Image, '/vision/arducam_display', SENSOR_QOS)

        self._cap = cv2.VideoCapture(device_id)
        if self._cap.isOpened():
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self._cap.set(cv2.CAP_PROP_FPS,          fps)
            self.get_logger().info(
                f"Arducam opened (device {device_id}, "
                f"{width}×{height} @ {fps} fps)."
            )
        else:
            self.get_logger().error(
                f"Arducam device {device_id} failed to open. "
                "Check the device index and USB connection."
            )

        self.create_timer(1.0 / fps, self._capture)

    def _capture(self) -> None:
        if not self._cap.isOpened():
            return

        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn(
                "Arducam read failed — skipping frame.",
                throttle_duration_sec=2.0,
            )
            return

        stamp = self.get_clock().now().to_msg()

        img_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp    = stamp
        img_msg.header.frame_id = 'arducam_optical_frame'

        self._raw_pub.publish(img_msg)
        self._display_pub.publish(img_msg)

    def destroy_node(self) -> None:
        if self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArducamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()