#!/usr/bin/env python3  
'''
Jonathan Freed

waypoint_local_node.py

Topic Subscription:
/odometry/local 

Topic Publishers:
/cmd_vel    geometry_msgs/Twist
'''
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)

def yaw_from_quaternion(q) -> float:
    """Yaw (rotation about z) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_to_pi(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__('waypoint_manager_node')

        # ------- Parameters --------------------
        self.declare_parameter('waypoints', [0.0, 0.0])
        self.declare_parameter('goal_tolerance', .3) #m
        self.declare_parameter('heading_spin_tolerance', math.pi/4) #rad
        self.declare_parameter('max_linear', .3) #m/s
        self.declare_parameter('max_angular', .8) #rad/s
        self.declare_parameter('frequency', 30) #hz
        self.declare_parameter('odom_timeout', .5) #s
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 1.5)

        waypoint_flat = self.get_parameter('waypoints').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.heading_tol = self.get_parameter('heading_spin_tolerance').value
        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value
        self.f = self.get_parameter('frequency').value
        self.odom_timeout = self.get_parameter('odom_timeout').value
        self.kp_lin = self.get_parameter('kp_linear').value
        self.kp_ang = self.get_parameter('kp_angular').value

        self.waypoints = [(waypoint_flat[i], waypoint_flat[i+1]) for i in range(0, len(waypoint_flat), 2)]

        # ------- State Variables ---------------
        self.have_pose = False
        self.x = self.y = self.yaw = 0.0
        self.last_odom_time = None
        self.wp_index = 0
        self.mission_done = False


        # ------- ROS Interfaces ----------------
        self.odom_sub = self.create_subscription(Odometry, '/odometry/local', self.odom_cb, QOS_PROFILE)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', QOS_PROFILE)

        self.timer = self.create_timer(1/self.f, self.control_loop)

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.last_odom_time = self.get_clock().now()
        self.have_pose = True

    def control_loop(self):
        if self.mission_done or not self.waypoints:
            self._stop() #publish zeros
            return
        if not self.have_pose or self._pose_stale():
            self._stop() #publish zeros
            return
        
        gx, gy = self.waypoints[self.wp_index]
        dx, dy = gx - self.x, gy - self.y
        distance = math.hypot(dx,dy)
        heading_err = wrap_to_pi(math.atan2(dy,dx)-self.yaw)

        if distance < self.goal_tol:
            self.get_logger().info(f'Reached waypoint {self.wp_index}, ({gx:.2f}, {gy:.2f})')
            self.wp_index += 1
            if self.wp_index >= len(self.waypoints):
                self.mission_done = True
                self._stop()
                self.get_logger().info('All waypoints reached, Stopping')
            return
        
        cmd = Twist()
        cmd.angular.z = clamp(self.kp_ang * heading_err, self.max_ang)
        if abs(heading_err) > self.heading_tol:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = clamp(self.kp_lin * distance, self.max_lin)
        
        self.cmd_vel_pub.publish(cmd)

    def _pose_stale(self) -> bool:
        if self.last_odom_time is None:
            return True
        dt = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9

        return dt > self.odom_timeout
    def _stop(self):
        self.cmd_vel_pub.publish(Twist())
    

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()