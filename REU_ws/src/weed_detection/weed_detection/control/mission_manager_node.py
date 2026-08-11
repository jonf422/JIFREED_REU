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
    #PATROL = auto()
    DONE = auto()
    FAILED = auto()

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        self.declare_parameter('coords', [0.0,0.0])
        self.declare_parameter('frame', 'odom')
        self.declare_parameter('required_topics', ['/imu/data', '/odom', '/odometry/local', '/odometry/gps', '/odometry/global', '/gps/heading'])

        self.required_topics = self.get_parameter('required_topics').value
        self.frame = self.get_parameter('frame').value
        self.waypoints = self._parse_coords(self.get_parameter('coords').value)
        self.wp_index = 0             # index of the waypoint currently being driven

        if self.waypoints:
            self.get_logger().info(
                f'Mission has {len(self.waypoints)} waypoint(s) in frame \'{self.frame}\': ' +
                ' -> '.join(f'({a},{b})' for a, b in self.waypoints))
        else:
            self.get_logger().info('No waypoints given — mission will complete immediately')

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
                if not self.waypoints:
                    self.transition(State.DONE)
                elif self.waypoint_client.server_is_ready():
                    self.transition(State.WAYPOINT)
                    self.send_current_waypoint()

        elif self.state == State.WAYPOINT:
            if self.action_done:
                if self.action_code != 0:
                    self.get_logger().error(
                        f'Waypoint {self.wp_index + 1}/{len(self.waypoints)} failed '
                        f'(code {self.action_code}) — aborting remaining waypoints')
                    self.transition(State.FAILED)
                else:
                    #this leg succeeded; move on to the next one, if any
                    self.wp_index += 1
                    if self.wp_index >= len(self.waypoints):
                        self.transition(State.DONE)
                    else:
                        self.send_current_waypoint()

        # TODO: IMplement visual servoing path patrol
        #elif self.state == State.PATROL:
        #    self.transition(State.DONE)
    

        elif self.state == State.DONE:
            self.get_logger().info(f'Mission COMPLETE — {self.wp_index}/{len(self.waypoints)} waypoints reached')
            self._timer.cancel()

        elif self.state == State.FAILED:
            self.get_logger().info(f'Mission FAILED — {self.wp_index}/{len(self.waypoints)} waypoints reached')
            self._timer.cancel()

    # -----------------------------------
    # Transition between states
    # -----------------------------------
    def transition(self, next_state):
        self.get_logger().info(f'{self.state.name} -> {next_state.name}')
        self.state = next_state

    # -----------------------------------
    # Parameter Parsing
    # -----------------------------------
    def _parse_coords(self, raw):
        """Flat [x1,y1, x2,y2, ...] -> [(x1,y1), (x2,y2), ...]. [0,0] means 'no waypoint'."""
        flat = [float(v) for v in (raw or [])]

        if len(flat) % 2 != 0:
            self.get_logger().error(
                f'coords must be a flat list of x,y pairs, got {flat} '
                f'({len(flat)} values) — treating as no-waypoint')
            return []

        pairs = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]

        #a lone [0.0, 0.0] is the documented "skip navigation" sentinel
        if pairs == [(0.0, 0.0)]:
            return []

        return pairs

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

    def _odom_check(self, msg):
        self._odom_seen = True

    # -----------------------------------
    # Navigation Functions
    # -----------------------------------
    def send_current_waypoint(self):
        """Dispatch the waypoint at self.wp_index as its own action goal."""
        self.send_waypoint_cmd(list(self.waypoints[self.wp_index]), self.frame)

    def send_waypoint_cmd(self, cmd, frame):
        self.action_done = False
        self.action_code = None

        goal = WaypointCommand.Goal()
        goal.coordinates = cmd
        goal.frame = frame
        self.get_logger().info(
            f"Sending waypoint {self.wp_index + 1}/{len(self.waypoints)}: {cmd} ({frame})")

        send_future = self.waypoint_client.send_goal_async(goal, self._on_feedback)
        send_future.add_done_callback(self.on_goal_response)

    
    # -----------------------------------
    # Result Handling
    # -----------------------------------
    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Waypoint {self.wp_index + 1}/{len(self.waypoints)} goal rejected")
            self.action_done = True
            self.action_code = -1        # sentinel: tick treats as failure
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(
            f"Waypoint {self.wp_index + 1}/{len(self.waypoints)} result: "
            f"code {result.return_code} ({result.message})")
        self.action_code = result.return_code
        self.action_done = True

    def _on_feedback(self, msg):
        feedback = msg.feedback
        self.get_logger().info(
            f'Waypoint {self.wp_index + 1}/{len(self.waypoints)} — Distance Remaining: {feedback.status} m',
            throttle_duration_sec=0.5)


def main():
    rclpy.init()
    node = MissionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
