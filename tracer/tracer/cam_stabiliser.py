#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math

class GimbalStabilizer(Node):
    def __init__(self):
        super().__init__('gimbal_stabilizer')
        
        # Subscribe to the 3D Odometry from Gazebo
        self.create_subscription(Odometry, '/model/tracer/odometry', self.odom_callback, 10)
        
        # Publisher for the bridged JointPositionController topic
        self.pitch_pub = self.create_publisher(Float64, '/model/tracer/camera/pitch_cmd', 10)
        self.offset = -45 * math.pi / 180  # -10 degrees in radians to pre-tilt the camera downwards
        self.get_logger().info("Gimbal Stabilizer Node Started")

    def odom_callback(self, msg):
        # 1. Extract Quaternion
        q = msg.pose.pose.orientation
        
        # 2. Convert Quaternion to Pitch (y-axis rotation)
        # Formula for Pitch in Euler from Quaternion
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) # vertical limit
        else:
            pitch = math.asin(sinp)

        # 3. Calculate Compensation
        # If drone pitches +5 deg, camera moves -5 deg relative to drone
        compensation_angle = -pitch - self.offset  # Invert pitch and apply offset for pre-tilt
        
        # 4. Publish Command
        cmd = Float64()
        cmd.data = compensation_angle
        self.pitch_pub.publish(cmd)

def main():
    rclpy.init()
    node = GimbalStabilizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()