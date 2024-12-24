#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from turtlesim.msg import Pose

class DrawCircle(Node):
    
    def __init__(self):
        super().__init__('draw_circle')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.get_logger().info("Draw Circle Node Started")
        # self.timer = self.create_timer(1.0, self.send_velocity_command)
    
    def send_velocity_command(self, x,z):
        msg = Twist()
        msg.linear.y = x
        msg.angular.z = z
        self.cmd_vel_pub.publish(msg)
    
    def pose_callback(self, msg: Pose):
        if msg.x >9 or msg.x < 2 or msg.y > 9 or msg.y < 2:
            self.send_velocity_command(1.0,1.0)
            self.get_logger().info(f"Turning from X: {msg.x}, Y: {msg.y}")
        else:
            self.send_velocity_command(1.0,0.0)

        

        

def main():
    rclpy.init()
    node = DrawCircle()
    node.get_logger().info("Hello World")
    rclpy.spin(node)
    rclpy.shutdown()