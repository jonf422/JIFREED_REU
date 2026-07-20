import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from weed_interfaces.action import WaypointCommand
from enum import Enum, auto

class State(Enum):
    INITIALIZING = auto()
    WAYPOINT = auto()
    PATROL = auto()
    DONE = auto()
    FAILED = auto()

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        self.declare_parameter('latitude', 0.0)
        self.declare_parameter('longitude', 0.0)

        self.lat = self.get_parameter('latitude').value
        self.lon = self.get_parameter('longitude').value

        #set state
        self.state = State.INITIALIZING

        self.action_done = False
        self.action_code = None
        
        self._timer = self.create_timer(1/10, self.tick) #10 hz

        self.waypoint_client = ActionClient(self, WaypointCommand, 'waypoint_command')

    # -----------------------------------
    # State Machine
    # -----------------------------------
    def tick(self):
        if self.state == State.INITIALIZING:
            if self.all_nodes_ready():
                if self.lat == 0.0 or self.lon == 0.0:
                    self.transition(State.PATROL)
                elif self.waypoint_client.server_is_ready():
                    self.transition(State.WAYPOINT)
                    self.send_waypoint_cmd([self.lat,self.lon])
        
        elif self.state == State.WAYPOINT:
            if self.action_done:
                if self.action_code == 0:
                    self.transition(State.PATROL)
                else:
                    self.transition(State.FAILED)
        
        # TODO: IMplement visual servoing path patrol
        elif self.state == State.PATROL:
            self.transition(State.DONE)
    

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
    # Initialization Check
    # -----------------------------------
    def all_nodes_ready(self):
        return True

    # -----------------------------------
    # Navigation Functions
    # -----------------------------------
    def send_waypoint_cmd(self, cmd):
        self.action_done = False
        self.action_code = None

        goal = WaypointCommand.Goal()
        goal.coordinates = cmd
        self.get_logger().info(f"Sending coordinates: {cmd}")

        send_future = self.waypoint_client.send_goal_async(goal)
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
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(
            f"Result: code {result.return_code} ({result.message})")
        self.action_code = result.return_code
        self.action_done = True


def main():
    rclpy.init()
    node = MissionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()