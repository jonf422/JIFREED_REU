#/usr/bin/env python3
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
/vision/realsense_depth      (sensor_msgs/Image,      z16)   — depth frames
/imu/data                     (sensor_msgs/Imu)             — gyro + accelerometer
/vision/realsense_camera_info  (sensor_msgs/CameraInfo)        — intrinsics

Parameters (ROS2 declare_parameter)
------------------------------------
width        int   640
height       int   480
fps          int   30
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import pyrealsense2 as rs

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)


class RealSenseNode(Node):

    def __init__(self):
        super().__init__('realsense_node')

        self.declare_parameter('width',  640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps',     30)

        w = self.get_parameter('width').value
        h = self.get_parameter('height').value
        fps = self.get_parameter('fps').value

        # Data cache
        self._latest_rgb = None
        self._latest_depth = None
        self._latest_gyro = None
        self._latest_accel = None

        # Publishers
        self._color_pub = self.create_publisher(Image, '/vision/realsense_color', SENSOR_QOS)
        self._depth_pub = self.create_publisher(Image, '/vision/realsense_depth', SENSOR_QOS)
        self._imu_pub = self.create_publisher(Imu, '/imu/data', SENSOR_QOS)
        self._info_pub = self.create_publisher(CameraInfo, '/vision/realsense_camera_info', 1)
        self._bridge = CvBridge()

    
        # Start RealSense pipeline
        self._pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63)
        cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        cam_profile = self._pipeline.start(cfg, self.pipeline_cb)

        # Align depth to color frame
        self._align = rs.align(rs.stream.color)
    

        # Extract and cache intrinsics for CameraInfo
        intr = (cam_profile
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

    # -------------------------------------------------------------------------
    # Realsense pipeline callbacks
    # -------------------------------------------------------------------------
    def pipeline_cb(self, frame) -> None:
        if frame.is_frameset():
            if self._latest_gyro is None or self._latest_accel is None:
                return

            aligned = self._align.process(frame)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()

            stamp = self.get_clock().now().to_msg()

            color_image = np.asanyarray(color_frame.get_data())
            color_msg   = self._bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            color_msg.header.stamp = stamp
            color_msg.header.frame_id = 'camera_link'

            depth_image = np.asanyarray(depth_frame.get_data())
            depth_msg   = self._bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
            depth_msg.header.stamp    = stamp
            depth_msg.header.frame_id = 'camera_link'

            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = 'camera_link'
            imu_msg.angular_velocity.x = self._latest_gyro.x
            imu_msg.angular_velocity.y = self._latest_gyro.y
            imu_msg.angular_velocity.z = self._latest_gyro.z
            imu_msg.linear_acceleration.x = self._latest_accel.x
            imu_msg.linear_acceleration.y = self._latest_accel.y
            imu_msg.linear_acceleration.z = self._latest_accel.z
            imu_msg.orientation_covariance[0] = -1.0

            self._camera_info.header.stamp = stamp

            self._color_pub.publish(color_msg)
            self._depth_pub.publish(depth_msg)
            self._imu_pub.publish(imu_msg)
            self._info_pub.publish(self._camera_info)

        elif frame.is_motion_frame():
            stream = frame.get_profile().stream_type()
            if stream == rs.stream.gyro:
                self._latest_gyro = frame.as_motion_frame().get_motion_data()
            elif stream == rs.stream.accel:
                self._latest_accel = frame.as_motion_frame().get_motion_data()
            else:
                self.get_logger().warning('Unexpected motion stream type')
        else:
            self.get_logger().warning('Unexpected frame type, skipping')

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
