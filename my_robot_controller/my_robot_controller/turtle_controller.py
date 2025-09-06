#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.side = 'right'
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # self.timers = self.create_timer(0.5, self.send_velocity)
        self.obstical_detection_sub = self.create_subscription(Pose, '/turtle1/pose', self.get_position, 10)
        self.get_logger().info('Turtle Controller Node has been started.')

    # def send_velocity(self, boundry_x, boundry_y):
    #     msg = Twist()
    #     msg.liner.x = 1.0
    #     if boundry_x >= 10.0 or boundry_x <= 0.0:
    #         msg.angular.z = 1.0
    #     elif boundry_y >= 10.0 or boundry_y <= 0.0:
    #         msg.angular.z = 1.0
    #     self.cmd_vel_pub.publish(msg)

    def get_position(self, msg: Pose):
        velocity = Twist()
        if msg.x >=9.0 or msg.x <=2.0:
            velocity.linear.x = 1.0
            velocity.angular.z = 0.9
        elif msg.y >=9.0 or msg.y <=2.0:
            velocity.linear.x = 1.0
            velocity.angular.z = 0.9
        else:
            velocity.linear.x = 3.0
        self.get_logger().info(f'Turtle Position -> x: {msg.x}, y: {msg.y}, theta: {msg.theta}')
        self.cmd_vel_pub.publish(velocity)
        if msg.x > 5.55 and msg.x < 5.58:
            self.get_logger().info('Turtle reached the middle of x-axis')
            if self.side == 'right':
                self.get_logger().info('Changing Pen Color to Red')
                self.call_set_pen_service(200, 0, 0, 3, 0)
                self.side = 'left'
            elif self.side == 'left':
                self.get_logger().info('Changing Pen Color to Purple')
                self.call_set_pen_service(100, 0, 100, 3, 0)
                self.side = 'right'
        

    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, '/turtle1/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))
        
    def callback_set_pen (self, future):
        try:
            response = future.result()
            self.get_logger().info('SetPen service call succeeded')
        except Exception as e:
            self.get_logger().error(f'SetPen service call failed: {e}')
        

def main(args=None):
    rclpy.init(args=args)

    node = TurtleController()
    rclpy.spin(node)

    rclpy.shutdown()