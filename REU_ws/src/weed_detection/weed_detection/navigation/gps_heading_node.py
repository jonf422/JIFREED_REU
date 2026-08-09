#!/usr/bin/env python3
'''
Jonathan Freed

gps_heading_node.py

Converts the skytraq PX1172RH's dual-antenna $--HDT heading (published by
nmea_navsat_driver as a compass bearing, clockwise from true north) into a
sensor_msgs/Imu absolute yaw suitable for robot_localization.

Topic Subscription:
/heading        geometry_msgs/QuaternionStamped (nmea_navsat_driver; yaw = radians(compass heading))

Topic Publishers:
/gps/heading    sensor_msgs/Imu
'''
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


def yaw_from_quaternion(q) -> float:
    """Yaw (rotation about z) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GpsHeadingNode(Node):
    def __init__(self):
        super().__init__('gps_heading_node')

        # heading_offset corrects for the antenna baseline not being mounted
        # parallel to base_link's forward axis (e.g. baseline runs side-to-side
        # or points backward). Measure by comparing reported heading to true
        # heading while stationary and set the difference here.
        self.declare_parameter('heading_offset', 0.0)  # rad
        self.declare_parameter('yaw_stddev', 0.0175)    # rad (~1 deg; dual-antenna RTK heading is tight)

        self.heading_offset = self.get_parameter('heading_offset').value
        yaw_stddev = self.get_parameter('yaw_stddev').value
        self.yaw_variance = yaw_stddev * yaw_stddev

        self.heading_sub = self.create_subscription(QuaternionStamped, '/heading', self._heading_cb, QOS_PROFILE)
        self.imu_pub = self.create_publisher(Imu, '/gps/heading', QOS_PROFILE)

    def _heading_cb(self, msg: QuaternionStamped):
        # nmea_navsat_driver encodes the raw compass bearing (clockwise from
        # true north) directly as yaw, so this is not yet an ENU yaw.
        q = msg.quaternion
        if math.isnan(q.x) or math.isnan(q.y) or math.isnan(q.z) or math.isnan(q.w):
            return
        
        compass_yaw = yaw_from_quaternion(msg.quaternion)
        enu_yaw = wrap_to_pi(math.pi / 2.0 - compass_yaw + self.heading_offset)

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'base_link'  # offset already applied, no further rotation needed

        qx, qy, qz, qw = quaternion_from_yaw(enu_yaw)
        out.orientation.x = qx
        out.orientation.y = qy
        out.orientation.z = qz
        out.orientation.w = qw
        out.orientation_covariance[8] = self.yaw_variance

        # angular velocity / linear acceleration are not measured here
        out.angular_velocity_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GpsHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
