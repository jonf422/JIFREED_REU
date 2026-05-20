#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class turtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_cb, 10)

        self.get_logger().info("turtle_controller Node")
    
    def pose_cb(self, pose:Pose):
        cmd = Twist()

        if pose.x > 9 or pose.x < 2 or pose.y > 9 or pose.y < 2:
            cmd.linear.x=1.0
            cmd.angular.z = .9
        else:
            cmd.linear.x=5.0
            cmd.angular.z = 0.0
        
        self.cmd_vel_publisher_.publish(cmd)
    

def main(args=None):
    rclpy.init(args=args)

    node = turtleControllerNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()