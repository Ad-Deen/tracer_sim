#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
import math

# ############################################################
# UI and Instruction Constants
# ############################################################
msg = """
Control Your Tracer Drone! (pynput Mode)
---------------------------
Moving around:      Rotation:          Vertical:
    w                   q (Left)           r (Up)
a   s   d               e (Right)          f (Down)
    x

w/x : +/- linear x (0.1)
a/d : +/- linear y (0.1)
r/f : +/- linear z (0.1)
q/e : +/- yaw rate (0.1 * pi)
s   : NEUTRAL (ALL ZERO)

CTRL-C to quit (Click terminal first)
"""

# ############################################################
# DroneTeleop Class Definition
# ############################################################
class DroneTeleop(Node):
    def __init__(self):
        super().__init__('drone_teleop')
        
        # Publisher to the namespaced topic
        self.publisher_ = self.create_publisher(Twist, '/tracer/cmd_vel', 10)
        
        # Initialize velocity state
        self.vx, self.vy, self.vz, self.yaw_rate = 0.0, 0.0, 0.0, 0.0
        
        # Set up a timer to publish at 10Hz
        self.timer = self.create_timer(0.1, self.publish_twist)
        
        # Start the keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()
        
        print(msg)

    def on_press(self, key):
        try:
            # Handle character keys
            k = key.char
            if k == 'w': self.vx += 0.1
            elif k == 'x': self.vx -= 0.1
            elif k == 'a': self.vy += 0.1
            elif k == 'd': self.vy -= 0.1
            elif k == 'r': self.vz += 0.1
            elif k == 'f': self.vz -= 0.1
            elif k == 'q': self.yaw_rate += (0.1 * math.pi)
            elif k == 'e': self.yaw_rate -= (0.1 * math.pi)
            elif k == 's':
                self.vx, self.vy, self.vz, self.yaw_rate = 0.0, 0.0, 0.0, 0.0
            
            # Print status line
            status = f"VX: {self.vx:.1f} | VY: {self.vy:.1f} | VZ: {self.vz:.1f} | Yaw: {self.yaw_rate:.2f}"
            print(f"\r{status}", end='', flush=True)
            
        except AttributeError:
            # Handle special keys (like space or esc) if needed
            pass

    def publish_twist(self):
        """ This runs every 0.1s regardless of key presses """
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.linear.z = self.vz
        twist.angular.z = self.yaw_rate
        self.publisher_.publish(twist)

# ############################################################
# Main Entry Point
# ############################################################
def main(args=None):
    rclpy.init(args=args)
    node = DroneTeleop()
    
    try:
        # rclpy.spin() now works perfectly because pynput is in its own thread
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency Stop on exit
        node.vx, node.vy, node.vz, node.yaw_rate = 0.0, 0.0, 0.0, 0.0
        node.publish_twist()
        node.get_logger().info("Shutting down teleop...")
        rclpy.shutdown()

if __name__ == '__main__':
    main()