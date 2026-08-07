#!/usr/bin/env python3
'''
Jonathan Freed

live_plot.py

Live view of how the robot is interpreting its own motion. Subscribes to
the command + odometry sources produced by weedbot.launch.py and
localization.launch.py and plots them against each other as they arrive,
so dead-reckoning, EKF fusion, and GPS correction can be compared directly
instead of inferred from a bag afterward.

Topic Subscriptions:
        /cmd_vel            geometry_msgs/Twist   commanded velocity
        /odom               nav_msgs/Odometry     wheel_odom_node (dead-reckoned from /cmd_vel, no feedback)
        /odometry/local     nav_msgs/Odometry     ekf_filter_node_odom (wheel odom + IMU, odom frame)
        /odometry/global    nav_msgs/Odometry     ekf_filter_node_map (+ GPS, map frame)
        /imu/data           sensor_msgs/Imu       raw yaw rate

Usage:
        ros2 run weed_detection live_plot_node
        ros2 run weed_detection live_plot_node --ros-args -p window_sec:=30.0
'''

import threading
import time
from collections import deque
from math import atan2, degrees

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import matplotlib.pyplot as plt
import matplotlib.animation as animation

QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def yaw_from_quaternion(q) -> float:
    #robot is planar (two_d_mode EKFs, wheel_odom_node only ever sets qz/qw), so the full roll/pitch/yaw decomposition collapses to this
    return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Trace:
    '''Parallel (time, value) ring buffers for one plotted quantity.'''

    def __init__(self, maxlen: int):
        self.t = deque(maxlen=maxlen)
        self.v = deque(maxlen=maxlen)

    def add(self, t: float, v) -> None:
        self.t.append(t)
        self.v.append(v)


class MotionLivePlotNode(Node):
    def __init__(self):
        super().__init__('motion_live_plot_node')

        self.declare_parameter('window_sec', 60.0)
        self.declare_parameter('history_points', 20000)
        self.window_sec = float(self.get_parameter('window_sec').value)
        history_points = int(self.get_parameter('history_points').value)

        self._lock = threading.Lock()
        self._t0 = time.monotonic()

        self.cmd_lin = Trace(history_points)
        self.cmd_ang = Trace(history_points)

        self.odom_lin = Trace(history_points)
        self.odom_ang = Trace(history_points)
        self.odom_yaw = Trace(history_points)
        self.odom_xy = Trace(history_points)

        self.local_lin = Trace(history_points)
        self.local_ang = Trace(history_points)
        self.local_yaw = Trace(history_points)
        self.local_xy = Trace(history_points)

        self.global_lin = Trace(history_points)
        self.global_ang = Trace(history_points)
        self.global_yaw = Trace(history_points)
        self.global_xy = Trace(history_points)

        self.imu_ang = Trace(history_points)

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, QOS_PROFILE)
        self.create_subscription(Odometry, '/odom',
                                  self._make_odom_cb(self.odom_lin, self.odom_ang, self.odom_yaw, self.odom_xy),
                                  QOS_PROFILE)
        self.create_subscription(Odometry, '/odometry/local',
                                  self._make_odom_cb(self.local_lin, self.local_ang, self.local_yaw, self.local_xy),
                                  QOS_PROFILE)
        self.create_subscription(Odometry, '/odometry/global',
                                  self._make_odom_cb(self.global_lin, self.global_ang, self.global_yaw, self.global_xy),
                                  QOS_PROFILE)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, QOS_PROFILE)

    def now(self) -> float:
        return time.monotonic() - self._t0

    def _cmd_vel_cb(self, msg: Twist) -> None:
        t = self.now()
        with self._lock:
            self.cmd_lin.add(t, msg.linear.x)
            self.cmd_ang.add(t, msg.angular.z)

    def _make_odom_cb(self, lin: Trace, ang: Trace, yaw: Trace, xy: Trace):
        def cb(msg: Odometry) -> None:
            t = self.now()
            with self._lock:
                lin.add(t, msg.twist.twist.linear.x)
                ang.add(t, msg.twist.twist.angular.z)
                yaw.add(t, degrees(yaw_from_quaternion(msg.pose.pose.orientation)))
                xy.add(t, (msg.pose.pose.position.x, msg.pose.pose.position.y))
        return cb

    def _imu_cb(self, msg: Imu) -> None:
        t = self.now()
        with self._lock:
            self.imu_ang.add(t, msg.angular_velocity.z)


def _xy(trace: Trace):
    xs = [p[0] for p in trace.v]
    ys = [p[1] for p in trace.v]
    return xs, ys


def main(args=None):
    rclpy.init(args=args)
    node = MotionLivePlotNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, ((ax_xy, ax_lin), (ax_ang, ax_yaw)) = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Weedbot Motion Interpretation')

    ax_xy.set_title('Position Estimate (XY)')
    ax_xy.set_xlabel('x (m)')
    ax_xy.set_ylabel('y (m)')
    ax_xy.set_aspect('equal', adjustable='datalim')
    ax_xy.grid(True)
    (line_odom_xy,) = ax_xy.plot([], [], label='/odom (wheel, dead-reckoned)', color='tab:gray')
    (line_local_xy,) = ax_xy.plot([], [], label='/odometry/local (wheel+IMU EKF)', color='tab:blue')
    (line_global_xy,) = ax_xy.plot([], [], label='/odometry/global (+GPS EKF)', color='tab:green')
    ax_xy.legend(loc='best', fontsize=8)

    ax_lin.set_title('Linear Velocity (x)')
    ax_lin.set_xlabel('t (s)')
    ax_lin.set_ylabel('m/s')
    ax_lin.grid(True)
    (line_cmd_lin,) = ax_lin.plot([], [], label='/cmd_vel commanded', color='black', linestyle='--')
    (line_odom_lin,) = ax_lin.plot([], [], label='/odom', color='tab:gray')
    (line_local_lin,) = ax_lin.plot([], [], label='/odometry/local', color='tab:blue')
    (line_global_lin,) = ax_lin.plot([], [], label='/odometry/global', color='tab:green')
    ax_lin.legend(loc='best', fontsize=8)

    ax_ang.set_title('Angular Velocity (yaw rate)')
    ax_ang.set_xlabel('t (s)')
    ax_ang.set_ylabel('rad/s')
    ax_ang.grid(True)
    (line_cmd_ang,) = ax_ang.plot([], [], label='/cmd_vel commanded', color='black', linestyle='--')
    (line_odom_ang,) = ax_ang.plot([], [], label='/odom', color='tab:gray')
    (line_local_ang,) = ax_ang.plot([], [], label='/odometry/local', color='tab:blue')
    (line_global_ang,) = ax_ang.plot([], [], label='/odometry/global', color='tab:green')
    (line_imu_ang,) = ax_ang.plot([], [], label='/imu/data (raw)', color='tab:red', alpha=0.6)
    ax_ang.legend(loc='best', fontsize=8)

    ax_yaw.set_title('Heading (yaw)')
    ax_yaw.set_xlabel('t (s)')
    ax_yaw.set_ylabel('deg')
    ax_yaw.grid(True)
    (line_odom_yaw,) = ax_yaw.plot([], [], label='/odom', color='tab:gray')
    (line_local_yaw,) = ax_yaw.plot([], [], label='/odometry/local', color='tab:blue')
    (line_global_yaw,) = ax_yaw.plot([], [], label='/odometry/global', color='tab:green')
    ax_yaw.legend(loc='best', fontsize=8)

    fig.tight_layout()

    all_lines = (line_odom_xy, line_local_xy, line_global_xy,
                 line_cmd_lin, line_odom_lin, line_local_lin, line_global_lin,
                 line_cmd_ang, line_odom_ang, line_local_ang, line_global_ang, line_imu_ang,
                 line_odom_yaw, line_local_yaw, line_global_yaw)

    def update(_frame):
        with node._lock:
            odom_x, odom_y = _xy(node.odom_xy)
            local_x, local_y = _xy(node.local_xy)
            global_x, global_y = _xy(node.global_xy)

            cmd_lin_t, cmd_lin_v = list(node.cmd_lin.t), list(node.cmd_lin.v)
            cmd_ang_t, cmd_ang_v = list(node.cmd_ang.t), list(node.cmd_ang.v)
            odom_lin_t, odom_lin_v = list(node.odom_lin.t), list(node.odom_lin.v)
            odom_ang_t, odom_ang_v = list(node.odom_ang.t), list(node.odom_ang.v)
            odom_yaw_t, odom_yaw_v = list(node.odom_yaw.t), list(node.odom_yaw.v)
            local_lin_t, local_lin_v = list(node.local_lin.t), list(node.local_lin.v)
            local_ang_t, local_ang_v = list(node.local_ang.t), list(node.local_ang.v)
            local_yaw_t, local_yaw_v = list(node.local_yaw.t), list(node.local_yaw.v)
            global_lin_t, global_lin_v = list(node.global_lin.t), list(node.global_lin.v)
            global_ang_t, global_ang_v = list(node.global_ang.t), list(node.global_ang.v)
            global_yaw_t, global_yaw_v = list(node.global_yaw.t), list(node.global_yaw.v)
            imu_ang_t, imu_ang_v = list(node.imu_ang.t), list(node.imu_ang.v)
            now = node.now()

        line_odom_xy.set_data(odom_x, odom_y)
        line_local_xy.set_data(local_x, local_y)
        line_global_xy.set_data(global_x, global_y)
        ax_xy.relim()
        ax_xy.autoscale_view()

        xlim = (max(0.0, now - node.window_sec), max(node.window_sec, now))

        line_cmd_lin.set_data(cmd_lin_t, cmd_lin_v)
        line_odom_lin.set_data(odom_lin_t, odom_lin_v)
        line_local_lin.set_data(local_lin_t, local_lin_v)
        line_global_lin.set_data(global_lin_t, global_lin_v)
        ax_lin.set_xlim(*xlim)
        ax_lin.relim()
        ax_lin.autoscale_view(scalex=False)

        line_cmd_ang.set_data(cmd_ang_t, cmd_ang_v)
        line_odom_ang.set_data(odom_ang_t, odom_ang_v)
        line_local_ang.set_data(local_ang_t, local_ang_v)
        line_global_ang.set_data(global_ang_t, global_ang_v)
        line_imu_ang.set_data(imu_ang_t, imu_ang_v)
        ax_ang.set_xlim(*xlim)
        ax_ang.relim()
        ax_ang.autoscale_view(scalex=False)

        line_odom_yaw.set_data(odom_yaw_t, odom_yaw_v)
        line_local_yaw.set_data(local_yaw_t, local_yaw_v)
        line_global_yaw.set_data(global_yaw_t, global_yaw_v)
        ax_yaw.set_xlim(*xlim)
        ax_yaw.relim()
        ax_yaw.autoscale_view(scalex=False)

        return all_lines

    ani = animation.FuncAnimation(fig, update, interval=200, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
