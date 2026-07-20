#!/usr/bin/env python3
'''
Jonathan Freed

tool_controller_node.py

Sends serial commands through usb to front tool upon recieving action command
'''
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import sys
import serial
import time
import serial.tools.list_ports
from weed_interfaces.action import ToolCommand

# ------------------------------------------------------------
#  CONFIG
#-------------------------------------------------------------
RETURN_CODES = {
    "0": "OK",
    "1": "Error: move unsafe (tool stuck or not homed)",
    "2": "Error: reserved (2)",
    "3": "Error: reserved (3)",
    "4": "Error: reserved (4)",
    "5": "Error: reserved (5)",
    "6": "Error: reserved (6)",
    "7": "Error: reserved (7)",
    "8": "Error: reserved (8)",
    "9": "Error: reserved (9)",
}
VALID_COMMANDS = {"home", "drill", "stop"}

TIMEOUT_PERIOD = {
    "home": 120,
    "drill": 60,
                  }

class ToolControllerNode(Node):
    def __init__(self):
        super().__init__('tool_controller_node')

        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('port', '/dev/tool_esp32')

        self.baud = self.get_parameter('baud_rate').value
        self.tool_port = self.get_parameter('port').value

        try:
            self.ser = serial.Serial(self.tool_port, self.baud, timeout=0.1)
            self.get_logger().info(f'Opened {self.tool_port} @ {self.baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open port: {self.tool_port} (tool): {e}')
            sys.exit(1)


        # -------------------------------------------------
        # Action Client
        # -------------------------------------------------
        self.action_server = ActionServer(
            node=self,
            action_type=ToolCommand,
            action_name='tool_command',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

    def goal_cb(self, goal_request):
        cmd = goal_request.command.strip().lower()
        if cmd not in ("home", "drill"):
            self.get_logger().warn(f"Rejecting unknown command: {cmd}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def cancel_cb(self, goal_handle):
        self.get_logger().warn("Cancel requested")
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        cmd = goal_handle.request.command.strip().lower()
        self.get_logger().info(f'Executing cmd: {cmd}')

        self.ser.reset_input_buffer()
        self.ser.write((cmd+'\n').encode())
        self.ser.flush()

        result = ToolCommand.Result()
        feedback = ToolCommand.Feedback()
        deadline = time.time() + TIMEOUT_PERIOD[cmd]

        while rclpy.ok():
            #handle cancelation
            if goal_handle.is_cancel_requested:
                self._send_stop()
                goal_handle.canceled()
                result.return_code = -1
                result.message = 'Canceled - Stop Sent'
                return result
            line = self.ser.readline().decode(errors="replace").strip()
            if line in RETURN_CODES:
                goal_handle.succeed()
                result.return_code = int(line)
                result.message = RETURN_CODES[line]
                return result
            elif line:
                # non-code chatter (e.g. "ESTOP") -> feedback, keep waiting
                feedback.status = line
                goal_handle.publish_feedback(feedback)

            # 3. timeout guard so a dead ESP32 can't hang the goal forever
            if time.time() > deadline:
                self._send_stop()
                goal_handle.abort()
                result.return_code = -2
                result.message = "Timeout waiting for tool"
                return result

            time.sleep(0.02)

    def _send_stop(self):
        self.ser.reset_input_buffer()
        self.ser.write(b"stop\n")
        self.ser.flush()
        self.get_logger().warn("Sent stop to tool")

def main():
    rclpy.init()
    node = ToolControllerNode()
    # MultiThreaded so cancel + execute aren't stuck on one thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    