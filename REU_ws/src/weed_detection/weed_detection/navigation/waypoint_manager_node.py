#!/usr/bin/env python3  
'''
Jonathan Freed

waypoint_manager_node.py

Topic Subscription:
/odometry/global

Topic Publishers:
/cmd_vel    geometry_msgs/Twist
'''
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from weed_interfaces.action import WaypointCommand
from robot_localization.srv import FromLL
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geographic_msgs.msg import GeoPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import threading
import time

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
        self.declare_parameter('goal_tolerance', .3) #m
        self.declare_parameter('accept_range', 20.0) #m
        self.declare_parameter('heading_spin_tolerance', math.pi/4) #rad
        self.declare_parameter('max_linear', .3) #m/s
        self.declare_parameter('max_angular', .8) #rad/s
        self.declare_parameter('frequency', 30.0) #hz
        self.declare_parameter('odom_timeout', .5) #s
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('fromll_timeout', 5.0) # s

        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.accept_range = self.get_parameter('accept_range').value
        self.heading_tol = self.get_parameter('heading_spin_tolerance').value
        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value
        self.f = self.get_parameter('frequency').value
        self.odom_timeout = self.get_parameter('odom_timeout').value
        self.kp_lin = self.get_parameter('kp_linear').value
        self.kp_ang = self.get_parameter('kp_angular').value
        self.fromll_timeout = self.get_parameter('fromll_timeout').value


        # ------- State Variables ---------------
        self.have_pose = False
        self.x = self.y = self.yaw = 0.0
        self.last_odom_time = None

        self._goal_handle = None      # active goal, or None when idle

        # -------- Thread locks -----------------
        self._goal_lock = threading.Lock()
        self._pose_lock = threading.Lock()

        # ------- ROS Interfaces ----------------
        self.from_ll_client = self.create_client(FromLL, '/fromLL', callback_group=MutuallyExclusiveCallbackGroup())

        self.odom_sub = self.create_subscription(Odometry, '/odometry/global', self.odom_cb, QOS_PROFILE)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', QOS_PROFILE)

        # ------- Action Client -------------
        self.action_server = ActionServer(
            node=self,
            action_type=WaypointCommand,
            action_name='waypoint_command',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

    def goal_cb(self, goal_request):
        coords = goal_request.coordinates

        #check validity of command
        if len(coords) != 2:
            self.get_logger().warn('Rejecting goal: coordinates must be [lat, lon]')
            return GoalResponse.REJECT
        lat, lon = coords[0], coords[1]
        if not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
            self.get_logger().warn(f'Rejecting goal: ({lat}, {lon}) out of range')
            return GoalResponse.REJECT
        with self._pose_lock:
            fresh = self.have_pose and not self._pose_stale()
        if not fresh:
            self.get_logger().warn('Rejecting goal: no fresh pose on /odometry/global')
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_handle is not None:
                self.get_logger().warn('Rejecting goal: another goal is active')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def cancel_cb(self, goal_handle):
        self.get_logger().warn("Cancel requested")
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        result = WaypointCommand.Result()
        feedback = WaypointCommand.Feedback()
        lat, lon = goal_handle.request.coordinates

        #TODO: convert lat,lon to map frame
        gx, gy = fromll()
        if gx is None or gy is None:
            goal_handle.abort()
            result.return_code = 1
            result.message = 'fromLL conversion failed'
            return result
        
        #calculate distance to goal, reject if greater than accept_range param
        with self._pose_lock:
            dist = math.hypot(gx - self.x, gy - self.y)
        if dist > self.accept_range:
            self.get_logger().warn(f'Canceling. Target is {dist:.1f}m away')
            goal_handle.abort()
            result.return_code = 1
            result.message = f'Target {dist:.1f}m away exceeds maximum of {self.accept_range}m'
            return result
        
        #set goal active
        with self._goal_lock:
            self._goal_handle = goal_handle
        self.get_logger().info(f'Navigating to ({lat:.6f},{lon:.6f}) -> map ({gx:.2f},{gy:.2f})')

        #control loop
        budget = 10 + 2*(dist/self.max_lin)
        deadline = time.time() + budget
        period = 1/self.f
        try:
            while rclpy.ok():
                #check cancel
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.return_code = -1
                    result.message = 'Canceled'
                    return result
                #check deadline
                if time.time() > deadline:
                    goal_handle.abort()
                    result.return_code = -2
                    result.message = "Timeout"
                    return result
                #check pose
                with self._pose_lock:
                    stale = not self.have_pose or self._pose_stale()
                    if not stale:
                        x, y, yaw = self.x, self.y, self.yaw
                if stale:
                    self._stop()
                    self.get_logger().warn('Pose stale — aborting')
                    goal_handle.abort()
                    result.return_code = 3
                    result.message = 'Lost pose'
                    return result

                   
                dx, dy = gx - x, gy - y
                distance = math.hypot(dx, dy)
                heading_err = wrap_to_pi(math.atan2(dy, dx) - yaw)

                if distance < self.goal_tol:
                    self.get_logger().info(f'Reached waypoint ({gx:.2f}, {gy:.2f})')
                    goal_handle.succeed()
                    result.return_code = 0
                    result.message = "Success"
                    return result

                feedback.status = (f'{distance:.2f} m to go, '
                                   f'heading err {heading_err:.2f} rad')
                goal_handle.publish_feedback(feedback)

                cmd = Twist()
                cmd.angular.z = clamp(self.kp_ang * heading_err, self.max_ang)
                cmd.linear.x = (0.0 if abs(heading_err) > self.heading_tol else clamp(self.kp_lin * distance, self.max_lin))
                self.cmd_vel_pub.publish(cmd)

                time.sleep(period)
        finally:
            self._stop()
            with self._goal_lock:
                self._goal_handle = None


    def odom_cb(self, msg: Odometry):
        with self._pose_lock:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
            self.last_odom_time = self.get_clock().now()
            self.have_pose = True
    
    def _from_ll(self, lat, lon):
        if not self.from_ll_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/fromLL service unavailable — is navsat_transform running?')
            return None, None

        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=float(lat), longitude=float(lon), altitude=0.0)

        future = self.from_ll_client.call_async(req)

        # Block this thread without spinning: another executor thread delivers the response.
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=self.fromll_timeout):
            future.cancel()
            self.get_logger().error('/fromLL call timed out')
            return None, None

        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'/fromLL call failed: {e}')
            return None, None

        return resp.map_point.x, resp.map_point.y

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
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()

