#!/usr/bin/env python3
import rclpy  #python library for ros2
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('first_node')
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("ROS2 listener: " + str(self.counter_))
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)  #Initialise ros2 communications

    node = MyNode()
    rclpy.spin(node)  #Keep node alive until it's killed manually

    rclpy.shutdown()  #Shutdown ros2 communicatios

if __name__ == '__main__':
    main()