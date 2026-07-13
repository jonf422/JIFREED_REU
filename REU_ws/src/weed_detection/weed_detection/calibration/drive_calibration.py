#!/usr/bin/env python3
'''
Jonathan Freed

drive_calibration.py

Node to calibrate speed_to_mps and half_track_width of drive_controller_node

publishes /cmd_vel geometry_msgs/Twist for 2 calibrations
    speed_to_mps (k):     actuator = v_wheel / k
    half_track_width (L): actuator = w * L / k

Corrections:
    straight test:  k_new = k_old * (D_measured / D_commanded)
    spin test:      L_new = L_old * (angle_commanded / angle_measured)

Topic Publications:
    /cmd_vel    geometry_msgs/Twist
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class DriveCalibration(Node):
    def __init__(self):
        super().__init__('drive_calibration_node')

        self.declare_parameter('desired_speed', 0.2)      # (m/s)
        self.declare_parameter('desired_distance', 1.0)   # (m)
        self.declare_parameter('desired_rad', 0.5)        # (rad/s)
        self.declare_parameter('desired_turns', 1)
        self.declare_parameter('pause_seconds', 10.0)
        self.declare_parameter('countdown', 3.0)

        self.v = self.get_parameter('desired_speed').value
        self.w = self.get_parameter('desired_rad').value
        self.turns = self.get_parameter('desired_turns').value
        self.pause = self.get_parameter('pause_seconds').value
        self.countdown = self.get_parameter('countdown').value
        dist = self.get_parameter('desired_distance').value

         # phase durations (guard against divide-by-zero)
        self.straight_dur = dist / self.v if self.v != 0 else 0.0
        self.rot_dur = abs(self.turns) * 2.0 * math.pi / abs(self.w) if self.w != 0 else 0.0

        # commanded values to report at the end
        self.cmd_distance = dist
        self.cmd_angle_deg = self.turns * 360.0

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0 / 20.0, self.tick)

        self.mode = 'countdown'
        self.phase_start = self.get_clock().now()
        self._last_count = None

        self.get_logger().info(
            f'straight {self.cmd_distance} m at {self.v} m/s ({self.straight_dur:.2f} s), '
            f'then {self.turns} turns at {self.w} rad/s ({self.rot_dur:.2f} s)')
        
    def elapsed(self) -> float:
        return (self.get_clock().now() - self.phase_start).nanoseconds * 1e-9
    
    def switch(self, mode: str) -> None:
        self.mode = mode
        self.phase_start = self.get_clock().now()

    def publish(self, vx: float = 0.0, wz: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.cmd_vel_pub.publish(msg)

    def tick(self):
        e = self.elapsed()

        if self.mode == 'countdown':
            self.publish() #publish zeroes for watchdog
            c = int(math.ceil(self.countdown - e))
            if c != self._last_count:
                self._last_count = c
                self.get_logger().info(f'starting in {c}...')
            if e >= self.countdown:
                self.get_logger().info('--- STRAIGHT ---')
                self.switch('straight')

        elif self.mode == 'straight':
            if e < self.straight_dur:
                self.publish(vx=self.v)
            else:
                self.get_logger().info(
                    f'straight done. commanded {self.cmd_distance} m. measure now '
                    f'({self.pause:.0f} s pause)')
                self.switch('pause')

        elif self.mode == 'pause':
            self.publish()  # zeros
            if e >= self.pause:
                self.get_logger().info('--- ROTATE ---')
                self.switch('rotate')

        elif self.mode == 'rotate':
            if e < self.rot_dur:
                self.publish(wz=self.w)
            else:
                self.publish()
                self.report()
                self.switch('done')

        elif self.mode == 'done':
            self.publish()  # hold zeros; Ctrl-C to exit


    def report(self) -> None:
        self.get_logger().info('--- DONE ---')
        self.get_logger().info(
            f'commanded distance = {self.cmd_distance} m  ->  '
            f'k_new = k_old * (D_measured / {self.cmd_distance})')
        self.get_logger().info(
            f'commanded angle = {self.cmd_angle_deg:.0f} deg  ->  '
            f'L_new = L_old * ({self.cmd_angle_deg:.0f} / angle_measured_deg)')


def main(args=None):
    rclpy.init(args=args)
    node = DriveCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish()  # stop on exit
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

