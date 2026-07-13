import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from weed_interfaces.action import ToolCommand
from enum import Enum, auto

class State(Enum):
    IDLE = auto()
    HOMING = auto()
    DRILLING = auto()
    DONE = auto()
    FAILED = auto()

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        #set state
        self.state = State.IDLE

        self.action_pending = False
        self.action_done = False
        self.action_code = None
        
        self._timer = self.create_timer(1/10, self.tick) #10 hz

        self.tool_client = ActionClient(self, ToolCommand, 'tool_command')

    # -----------------------------------
    # State Machine
    # -----------------------------------
    def tick(self):
        if self.state == State.IDLE:
            if self.tool_client.server_is_ready():
                self.transition(State.HOMING)
                self.send_tool_cmd('home')
        
        elif self.state == State.HOMING:
            if self.action_done:
                if self.action_code == 0:
                    self.transition(State.DRILLING)
                    self.send_tool_cmd('drill')
                else:
                    self.transition(State.FAILED)

        elif self.state == State.DRILLING:
            if self.action_done:
                if self.action_code == 0:
                    self.transition(State.DONE)
                else:
                    self.transition(State.FAILED)

        elif self.state == State.DONE:
            self.get_logger().info('Mission COMPLETE')
            self._timer.cancel()

        elif self.state == State.FAILED:
            self.get_logger().info('Mission FAILED')
            self._timer.cancel()

    # -----------------------------------
    # Transition between states
    # -----------------------------------
    def transition(self, next_state):
        self.get_logger().info(f'{self.state.name} -> {next_state.name}')
        self.state = next_state


    # -----------------------------------
    # Drill Functions
    # -----------------------------------
    def send_tool_cmd(self, cmd):
        self.action_pending = True
        self.action_done = False
        self.action_code = None

        goal = ToolCommand.Goal()
        goal.command = cmd
        self.get_logger().info(f"Dispatching '{cmd}'")

        send_future = self.tool_client.send_goal_async(goal)
        send_future.add_done_callback(self.on_goal_response)

    
    # -----------------------------------
    # Result Handling
    # -----------------------------------
    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.action_done = True
            self.action_code = -1        # sentinel: tick treats as failure
            self.action_pending = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(
            f"Result: code {result.return_code} ({result.message})")
        self.action_code = result.return_code
        self.action_done = True
        self.action_pending = False


def main():
    rclpy.init()
    node = MissionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()