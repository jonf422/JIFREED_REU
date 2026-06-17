#!/usr/bin/env python3

# SEED25
'''
Node that controls motor commands via terminal inputs. 
Simple remote control.

Up arrow: Move forward
Down arrow: Move backward
Right arrow: Pivot right
Left arrow: Pivot left
'''



import rclpy
from rclpy.node import Node
import sys
import select
import termios
import tty
import atexit

# Import your custom modules from this package
import sabertooth as st
from PID import PID

# Terminal Arrow Key Escape Sequences
UP = '\x1b[A'
DOWN = '\x1b[B'
RIGHT = '\x1b[C'
LEFT = '\x1b[D'
QUIT = '\x03'  # Ctrl + C

class SimpleTeleopNode(Node):
    def __init__(self):
        super().__init__('simple_teleop_node')
        
        self.get_logger().info('Initializing Simple Teleop...')
        
        # Base speed for the teleop commands
        self.DRIVE_SPEED = 50 
        self.TURN_SPEED = 50

        # Initialize Motors
        try:
            self.motor = st.SaberToothMotorDriver(True, True)
            self.get_logger().info("Sabertooth Initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize motors: {e}")
            sys.exit(1)
            
        atexit.register(self.motor.all_motors_off)

        # Initialize PID (Included for future closed-loop scaling if desired)
        self.pid = PID(Kp=1.0, Ki=5.0, Kd=5.0, Ts=0.1, umax=100, umin=-100)

    def destroy_node(self):
        self.motor.all_motors_off()
        super().destroy_node()


def get_key(settings, timeout=0.1):
    """Reads a single keypress from the terminal without requiring Enter."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    
    if rlist:
        key = sys.stdin.read(1)
        # If it's an escape character, read the next two bytes for arrow keys
        if key == '\x1b':
            key += sys.stdin.read(2)
    else:
        key = ''
        
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTeleopNode()

    # Save original terminal settings so we don't break the user's console
    settings = termios.tcgetattr(sys.stdin)

    print("\n" + "="*40)
    print(" 🕹️  SIMPLE TELEOP CONTROLLER")
    print("="*40)
    print("  Use Arrow Keys to move.")
    print("  Release keys to stop.")
    print("  Press Ctrl+C to exit.")
    print("="*40 + "\n")

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.1)

            if key == QUIT:
                break
            
            # Map keys to motor speeds (Left Wheel, Right Wheel)
            if key == UP:
                node.motor.updateMotorSpeed(node.DRIVE_SPEED, node.DRIVE_SPEED)
            elif key == DOWN:
                node.motor.updateMotorSpeed(-node.DRIVE_SPEED, -node.DRIVE_SPEED)
            elif key == LEFT:
                node.motor.updateMotorSpeed(-node.TURN_SPEED, node.TURN_SPEED)
            elif key == RIGHT:
                node.motor.updateMotorSpeed(node.TURN_SPEED, -node.TURN_SPEED)
            else:
                # If no valid key is actively being pressed, stop the motors
                node.motor.updateMotorSpeed(0, 0)
                
            # Allow ROS 2 to process any internal callbacks
            rclpy.spin_once(node, timeout_sec=0.0)

    except Exception as e:
        node.get_logger().error(f"Teleop error: {e}")
        
    finally:
        # Restore normal terminal behavior and shut down
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()