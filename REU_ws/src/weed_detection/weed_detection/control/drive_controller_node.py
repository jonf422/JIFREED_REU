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
    max_actuator        (float, default 30.0)   maximum Sabertooth command value
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from weed_detection.utils.sabertooth import SaberToothMotorDriver

import atexit
import time


class DriveControllerNode(Node):
    def __init__(self):
        super().__init__('drive_controller')

        # Kinematic parameters
        self.declare_parameter('half_track_width', 0.178)
        self.declare_parameter('speed_to_mps',     0.02)
        self.declare_parameter('max_actuator',     30.0)

        self._L            = self.get_parameter('half_track_width').value
        self._mps_per_motor_unit = self.get_parameter('speed_to_mps').value
        self._max_actuator = self.get_parameter('max_actuator').value

        #Initialize Sabertooth Motor Driver
        try:
            self._motors = SaberToothMotorDriver(True, True)
            self.get_logger().info("Sabertooth Motor Driver Initialized")
        except Exception as e:
            self.get_logger().error(f"Sabertooth Motor Driver Failed to Initialize:\n{e}")
            raise
        
        #register motors off as exit function for safety
        atexit.register(self._motors.all_motors_off)

        #Subscription to Nav2 velocity commands
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        
        #Watchdog to monitor for silent commands
        self._motors_stopped = True
        self._last_msg_t = time.time() - 1.0
        self.create_timer(.1, self.safety_check)

    def cmd_vel_cb(self, msg) -> None:
        self._last_msg_t = time.time()
        self._motors_stopped = False

        #recieve Twist cmd
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

        # temporary debug — remove before field use
        self.get_logger().info(f'v={v:.3f} w={w:.3f} | L={left_cmd:.1f} R={right_cmd:.1f}')

        #Send command to Sabertooth Driver
        self._motors.updateMotorSpeed(left_cmd, right_cmd)

    #Stop motors if velocity commands time out
    def safety_check(self) -> None:
        if time.time() - self._last_msg_t > .5:
            if not self._motors_stopped:
                self._motors.updateMotorSpeed(0,0)
                self._motors_stopped = True
                self.get_logger().warning('/cmd_vel timed out, stopping motors')

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

        
