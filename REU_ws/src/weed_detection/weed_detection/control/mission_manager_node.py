import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from weed_interfaces.action import WaypointCommand
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from enum import Enum, auto

ODOM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

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
        self.declare_parameter('required_topics', ['/imu/data', '/odom', '/odometry/local', '/odometry/gps', '/odometry/global'])

        self.lat = self.get_parameter('latitude').value
        self.lon = self.get_parameter('longitude').value
        self.required_topics = self.get_parameter('required_topics').value

        #set state
        self.state = State.INITIALIZING

        self.action_done = False
        self.action_code = None

        self._odom_seen = False

        self.odom_check = self.create_subscription(Odometry, '/odometry/global', self._odom_check, ODOM_QOS)
        
        self._timer = self.create_timer(1/10, self.tick) #10 hz

        self.waypoint_client = ActionClient(self, WaypointCommand, 'waypoint_command')

    # -----------------------------------
    # State Machine
    # -----------------------------------
    def tick(self):
        if self.state == State.INITIALIZING:
            if self.all_nodes_ready():
                if self.lat == 0.0 and self.lon == 0.0:
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
        missing_topic = [t for t in self.required_topics if self.count_publishers(t) == 0]
        missing_action = [name for name, client in [('waypoint_command', self.waypoint_client)] if not client.server_is_ready()]

        if missing_topic or missing_action:
            if missing_topic:
                self.get_logger().warn(f'Waiting on Topics: {", ".join(missing_topic)}', throttle_duration_sec=2.0)
            if missing_action:
                self.get_logger().warn(f'Waiting on Action Clients: {", ".join(missing_action)}', throttle_duration_sec=2.0)
            return False

        if not self._odom_seen:
            self.get_logger().warn('Publisher present but no /odometry/global message yet — waiting for GPS lock', throttle_duration_sec=2.0)
            return False
    
        return True

    def _odom_probe_cb(self, msg):
        self._odom_seen = True

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