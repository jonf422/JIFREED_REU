#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial

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
        
        #typically don't want to call service so frequently
        self.call_set_pen_service(255*pose.x/12, 255*pose.y/12, 255*abs(pose.theta)/12, 5, 0)
        
        self.cmd_vel_publisher_.publish(cmd)

    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, "/turtle1/set_pen")

        while not client.wait_for_service(1.0):
            self.get_logger().warning("Waiting for service...")
        
        request = SetPen.Request()
        request.r = int(r)
        request.g = int(g)
        request.b = int(b)
        request.width = int(width)
        request.off = int(off)

        future = client.call_async(request)
        future.add_done_callback(partial(self.set_pen_cb))

    def set_pen_cb(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

    

def main(args=None):
    rclpy.init(args=args)

    node = turtleControllerNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()