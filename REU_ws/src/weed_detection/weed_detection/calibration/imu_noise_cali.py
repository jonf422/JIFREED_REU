#!/usr/bin/env python3
"""
imu_noise_cali.py

Subscribes to /imu/data, collects samples while the robot is STATIONARY,
and reports the variance of each axis. Use the variances as starting
diagonal covariance values in realsense_node.py.

Usage:
    1. Place robot on a stable, level surface. Do NOT touch it.
    2. ros2 run weed_detection measure_imu_noise      (or: python3 measure_imu_noise.py)
    3. Let it collect (default 1000 samples ~5s at 200Hz), read the printout.

Note: at rest, the accelerometer reads gravity (~9.8 on one axis). We report
VARIANCE (spread around the mean), not the mean, so the gravity offset doesn't
matter — variance captures the noise, which is what the EKF needs.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import numpy as np

# Must match realsense_node.py's SENSOR_QOS (BEST_EFFORT) or the
# subscription will silently never connect to the publisher.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)


class ImuNoiseMeasurer(Node):
    def __init__(self, target_samples=1000):
        super().__init__('imu_noise_measurer')
        self.target = target_samples
        self.gyro = []   # rows of [x, y, z]
        self.accel = []
        self.create_subscription(Imu, '/imu/data', self.cb, SENSOR_QOS)
        self.get_logger().info(
            f'Collecting {target_samples} samples — keep the robot PERFECTLY STILL...'
        )

    def cb(self, msg: Imu):
        self.gyro.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        self.accel.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        n = len(self.gyro)
        if n % 200 == 0:
            self.get_logger().info(f'  {n}/{self.target}')
        if n >= self.target:
            self.report()
            rclpy.shutdown()

    def report(self):
        g = np.array(self.gyro)
        a = np.array(self.accel)
        gv = g.var(axis=0)   # variance per axis
        av = a.var(axis=0)

        print('\n' + '=' * 60)
        print(f'Samples: {len(self.gyro)}')
        print('\nANGULAR VELOCITY (gyro) variance [(rad/s)^2]:')
        print(f'  x={gv[0]:.3e}  y={gv[1]:.3e}  z={gv[2]:.3e}')
        print('\nLINEAR ACCELERATION (accel) variance [(m/s^2)^2]:')
        print(f'  x={av[0]:.3e}  y={av[1]:.3e}  z={av[2]:.3e}')
        print('\n--- suggested diagonal covariance arrays ---')
        # round up slightly for safety margin; EKF prefers honest-to-slightly-pessimistic
        print('angular_velocity_covariance = [')
        print(f'    {gv[0]:.3e}, 0.0, 0.0,')
        print(f'    0.0, {gv[1]:.3e}, 0.0,')
        print(f'    0.0, 0.0, {gv[2]:.3e}]')
        print('linear_acceleration_covariance = [')
        print(f'    {av[0]:.3e}, 0.0, 0.0,')
        print(f'    0.0, {av[1]:.3e}, 0.0,')
        print(f'    0.0, 0.0, {av[2]:.3e}]')
        print('=' * 60)
        print('\nNOTE: these are at-rest (static) noise floors. Real motion adds')
        print('vibration noise, so consider inflating 2-5x for field use.')


def main(args=None):
    rclpy.init(args=args)
    node = ImuNoiseMeasurer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # shutdown handled in report(); guard double-shutdown
    try:
        node.destroy_node()
    except Exception:
        pass


if __name__ == '__main__':
    main()