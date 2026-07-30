#!/usr/bin/env python3
'''
Jonathan Freed

drive_controller_node.py

Creates object of Sabertooth Motor Driver. Receives velocity commands from navigation. 
Translates velocity commands into left/right motor speeds.


Topic subscriptions:
/cmd_vel    geometry_msgs/Twist     (linear.x, angular.z)

Parameters:
    half_track_width    (float, default 0.178)  half distance between wheels in meters
    mps_per_actuator_unit (float, default 0.02) m/s per Sabertooth actuator unit
    max_actuator        (float, default 100.0)   maximum Sabertooth command value
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from weed_detection.utils.sabertooth import SaberToothMotorDriver
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import atexit
import time

QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)


class DriveControllerNode(Node):
    def __init__(self):
        super().__init__('drive_controller')

        # Kinematic parameters
        self.declare_parameter('half_track_width', 0.178)
        self.declare_parameter('speed_to_mps',     0.01)
        self.declare_parameter('max_actuator',     100.0)

        self._L = self.get_parameter('half_track_width').value
        self._mps_per_motor_unit = self.get_parameter('speed_to_mps').value
        self._max_actuator = self.get_parameter('max_actuator').value

        #Initialize Sabertooth Motor Driver
        try:
            self._motors = SaberToothMotorDriver(True, True, call_rate_hz=30.0)
            self.get_logger().info("Sabertooth Motor Driver Initialized")
        except Exception as e:
            self.get_logger().error(f"Sabertooth Motor Driver Failed to Initialize:\n{e}")
            raise
        
        #register motors off as exit function for safety
        atexit.register(self._motors.all_motors_off)

        #Subscription to Nav2 velocity commands
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, QOS_PROFILE)
        
        #Watchdog to monitor for silent commands
        self._motors_stopped = False
        self._last_msg_t = time.time() - 1.0
        self.create_timer(.1, self.safety_check)

    def cmd_vel_cb(self, msg) -> None:
        self._last_msg_t = time.time()
        self._motors_stopped = False

        #recieve Twist cm
        v = msg.linear.x #m/s forward
        w = msg.angular.z #rad/s (ccw positive)

        #set wheel velocities
        v_left = v - w * self._L
        v_right = v + w * self._L

        #convert m/s -> actuator units
        left_cmd = v_left / self._mps_per_motor_unit
        right_cmd = v_right / self._mps_per_motor_unit

        #scale to max actuator input if either exceeds limit
        peak = max(abs(left_cmd), abs(right_cmd))
        if peak > self._max_actuator:
            left_cmd *= self._max_actuator / peak
            right_cmd *= self._max_actuator / peak

        #debug logger

        #Send command to Sabertooth Driver
        self._motors.updateMotorSpeed(left_cmd, right_cmd)

    #Stop motors if velocity commands time out
    def safety_check(self) -> None:
        if time.time() - self._last_msg_t > 0.5:
            if not self._motors_stopped:
                self._motors.updateMotorSpeed(0, 0) # one ramp step per tick
                if (abs(self._motors.get_motor1_speed()) < 1e-3 and abs(self._motors.get_motor2_speed()) < 1e-3):
                    self._motors.all_motors_off() # hard off once at zero
                    self._motors_stopped = True
                    self.get_logger().warning('/cmd_vel timed out, motors stopped')

    def destroy_node(self):
        self._motors.updateMotorSpeed(0,0)
        self._motors.all_motors_off()
        return super().destroy_node()
    

def main(args=None):
    rclpy.init(args=args)
    node = DriveControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()

        
