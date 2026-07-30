#!/usr/bin/env python3
'''
Jonathan Freed

wheel_odom_node.py

Open loop wheel odometry for robot localization
Estimates motion from commanded velocities

Topic Subscriptions:
        /cmd_vel    geometry_msgs/Twist (linear.x m/s, angular.z rad/s)

Topic Publications:
        /odom       nav_msgs/Odometry

'''

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
 
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry

import numpy as np

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

def yaw_to_quaternion(yaw: float) -> Quaternion:
    #2D yaw -> quaternion (only z/w are nonzero for planar rotation).
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = np.sin(yaw * 0.5)
    q.w = np.cos(yaw * 0.5)
    return q

class WheelOdomNode(Node):
    def __init__(self):
        super().__init__('wheel_odom_node')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v = 0.0
        self.w = 0.0

        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, qos)

        self.create_timer(0.05, self.update)

    def cmd_vel_cb(self, msg: Twist) -> None:
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self) -> None:
        t = self.get_clock().now()
        dt = (t - self.last_time).nanoseconds * 1e-9
        self.last_time = t

        if dt <=0:
            return
        
        #differential drive kinematics
        self.theta += self.w * dt
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt

        #create odom msg
        odom = Odometry()
        odom.header.stamp = t.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        #big covariances means don't trust readings
        big = 1e6
        odom.pose.covariance = [
            big, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, big, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, big, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, big, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, big, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, big,
        ]
        odom.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,    # vx variance
            0.0,  big, 0.0, 0.0, 0.0, 0.0,    # vy unused (diff drive)
            0.0,  0.0, big, 0.0, 0.0, 0.0,
            0.0,  0.0, 0.0, big, 0.0, 0.0,
            0.0,  0.0, 0.0, 0.0, big, 0.0,
            0.0,  0.0, 0.0, 0.0, 0.0, 0.10,   # vyaw variance
        ]
 
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()