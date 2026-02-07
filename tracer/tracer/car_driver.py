#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class ChaosDriver(Node):
    def __init__(self):
        super().__init__('chaos_driver')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Start the first cycle
        self.schedule_next_move()

    def schedule_next_move(self):
        # Pick a random delay between 2 and 6 seconds
        random_delay = random.uniform(2.0, 6.0)
        
        # Create a "one-shot" timer by making sure we destroy it in the callback
        self.timer = self.create_timer(random_delay, self.timer_callback)
        self.get_logger().info(f'Next unpredictable move in {random_delay:.2f} seconds...')

    def timer_callback(self):
        # 1. First, stop the current timer so it doesn't fire again
        self.timer.cancel()
        self.destroy_timer(self.timer)

        # 2. Execute the random behavior
        msg = Twist()
        msg.linear.x = random.uniform(0.0, 2.0)
        msg.angular.z = random.uniform(-1.5, 1.5)
        self.publisher_.publish(msg)
        self.get_logger().info(f'ACTING: v={msg.linear.x:.2f}, w={msg.angular.z:.2f}')

        # 3. Schedule the NEXT random interval
        self.schedule_next_move()

def main(args=None):
    rclpy.init(args=args)
    node = ChaosDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()