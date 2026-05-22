"""
realsense_node.py

The single owner of the Intel RealSense pipeline.

All other nodes that need RealSense frames subscribe to the topics published
here rather than opening the device directly.  This prevents the
"device already opened" runtime error that occurs when multiple nodes each
try to call rs.pipeline().start() on the same USB device.

Topics published
----------------
/vision/realsense_color        (sensor_msgs/Image,      bgr8)  — colour frames
/vision/realsense_camera_info  (sensor_msgs/CameraInfo)        — intrinsics
/vision/realsense_display      (sensor_msgs/Image,      bgr8)  — alias for display_node
                                 (same data, separate publisher so display_node
                                  can use a depth-2 queue without affecting
                                  the processing pipeline)

Parameters (ROS2 declare_parameter)
------------------------------------
width        int   640
height       int   480
fps          int   30
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)

import numpy as np
import pyrealsense2 as rs


class RealSenseNode(Node):

    def __init__(self):
        super().__init__('realsense_node')

        self.declare_parameter('width',  640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps',     30)

        w   = self.get_parameter('width').value
        h   = self.get_parameter('height').value
        fps = self.get_parameter('fps').value

        # Publishers
        self._bridge = CvBridge()

        self._color_pub   = self.create_publisher(Image, '/vision/realsense_color',      SENSOR_QOS)
        self._display_pub = self.create_publisher(Image, '/vision/realsense_display',     SENSOR_QOS)
        self._info_pub    = self.create_publisher(CameraInfo, '/vision/realsense_camera_info', 1)

        # Start RealSense pipeline
        self._pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        profile = self._pipeline.start(cfg)

        # Extract and cache intrinsics for CameraInfo
        intr = (profile
                .get_stream(rs.stream.color)
                .as_video_stream_profile()
                .get_intrinsics())

        self._camera_info = self._build_camera_info(intr, w, h)

        self.get_logger().info(
            f"RealSense pipeline started ({w}×{h} @ {fps} fps). "
            f"fx={intr.fx:.1f}  fy={intr.fy:.1f}  "
            f"cx={intr.ppx:.1f}  cy={intr.ppy:.1f}  "
            f"dist={intr.coeffs}"
        )

        # Capture loop at the stream's native rate
        self.create_timer(1.0 / fps, self._capture)

    # -----------------------------------------------------------------------

    @staticmethod
    def _build_camera_info(intr, w: int, h: int) -> CameraInfo:
        ci = CameraInfo()
        ci.width  = w
        ci.height = h
        ci.distortion_model = 'plumb_bob'
        ci.d = list(intr.coeffs)          # [k1, k2, p1, p2, k3]
        ci.k = [
            intr.fx, 0.0,     intr.ppx,
            0.0,     intr.fy, intr.ppy,
            0.0,     0.0,     1.0,
        ]
        ci.r = [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
        ci.p = [
            intr.fx, 0.0,     intr.ppx, 0.0,
            0.0,     intr.fy, intr.ppy, 0.0,
            0.0,     0.0,     1.0,      0.0,
        ]
        return ci

    def _capture(self) -> None:
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=50)
        except RuntimeError:
            return  # No frame arrived within the timeout — skip this tick

        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        frame = np.asanyarray(color_frame.get_data())
        stamp = self.get_clock().now().to_msg()

        # Build image message (shared header)
        img_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp    = stamp
        img_msg.header.frame_id = 'realsense_color_optical_frame'

        # Publish on the processing topic (queue-2 keeps backpressure low)
        self._color_pub.publish(img_msg)

        # Display topic — same message, subscribers use it for visualisation
        self._display_pub.publish(img_msg)

        # Publish CameraInfo at the same rate (latched-style: always fresh)
        self._camera_info.header.stamp    = stamp
        self._camera_info.header.frame_id = img_msg.header.frame_id
        self._info_pub.publish(self._camera_info)

    # -----------------------------------------------------------------------

    def destroy_node(self) -> None:
        self.get_logger().info("Stopping RealSense pipeline.")
        try:
            self._pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()