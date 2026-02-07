#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3  


# ############################################################
# UI and Instruction Constants
# ############################################################
msg = """
Control Your Tracer Drone! (Manual & Auto Mode)
-----------------------------------------------
Mode Selection:
    m                   TOGGLE MODE (MANUAL / AUTO)

Manual Controls (MANUAL Mode Only):
    w/x : +/- linear x (Forward/Back)
    a/d : +/- linear y (Left/Right)
    r/f : +/- linear z (Up/Down)
    q/e : +/- yaw rate (Rotation)
    s   : NEUTRAL (ALL ZERO)

CTRL-C to quit
"""

class DroneTeleop(Node):
    def __init__(self):
        super().__init__('drone_teleop')
        
        self.publisher_ = self.create_publisher(Twist, '/tracer/cmd_vel', 10)
        
        # Subscribe to the 3D Odometry from Gazebo
        self.create_subscription(Odometry, '/model/tracer/odometry', self.odom_callback, 10)
        
        #Subscribe to target errors
        self.create_subscription(Vector3, '/target_errors', self.error_callback, 10)

        # Store the current altitude for potential use in Auto Mode
        self.current_altitude = 0.0
        self.vx_error = 0.0
        self.yaw_error = 0.0


        # Mode state: "MANUAL" or "AUTO"
        self.mode = "MANUAL"
        
        #altitude hold variables PID
        self.altitude_setpoint = 3.0  # Desired altitude in meters
        self.alt_kp = 1.0
        # self.alt_kd = 0.001
        # self.alt_ki = 0.0005
        self.alt_error = 0.0
        # self.last_alt_error = 0.0

        #yaw control variables
        self.kp_pursuit_yaw = 1.0

        #vx control variables
        self.kp_pursuit_vx = 2.0


        # Velocity state (Manual)
        self.vx, self.vy, self.vz, self.yaw_rate = 0.0, 0.0, 0.0, 0.0
        
        # Automated Agent values (Placeholders)
        self.auto_vx, self.auto_vy, self.auto_vz, self.auto_yaw = 0.0, 0.0, 0.0, 0.0

        self.timer = self.create_timer(0.1, self.publish_twist)
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()
        
        print(msg)

    def odom_callback(self, msg):
        # 1. Extract Altitude (z) only
        self.current_altitude = msg.pose.pose.position.z

    def error_callback(self, msg):
        # This callback receives target errors from the RedCarDetector
        self.vx_error = msg.x  # Vertical error (Vx)
        self.yaw_error = msg.y # Horizontal error (Yaw)
        # self.detected = msg.z > 0.5  # target_found flag (1.0 = True, 0.0 = False)
    def on_press(self, key):
        """ Handles ONE-TIME mode toggles and manual input increments """
        try:
            k = key.char
            
            # 1. MODE TOGGLE (One-time trigger)
            if k == 'm':
                self.mode = "AUTO" if self.mode == "MANUAL" else "MANUAL"
                # Reset all velocities for safety transition
                self.vx = self.vy = self.vz = self.yaw_rate = 0.0
                self.auto_vx = self.auto_vy = self.auto_vz = self.auto_yaw = 0.0
                print(f"\n[MODE CHANGE] Switched to: {self.mode}")
                return # Exit early

            # 2. MANUAL CONTROL (Only updates values, doesn't publish)
            if self.mode == "MANUAL":
                if k == 'w': self.vx += 0.1
                elif k == 'x': self.vx -= 0.1
                elif k == 'a': self.vy += 0.1
                elif k == 'd': self.vy -= 0.1
                elif k == 'r': self.vz += 0.1
                elif k == 'f': self.vz -= 0.1
                elif k == 'q': self.yaw_rate += (0.1 * math.pi)
                elif k == 'e': self.yaw_rate -= (0.1 * math.pi)
                elif k == 's':
                    self.vx = self.vy = self.vz = self.yaw_rate = 0.0
                
                status = f"MODE: {self.mode} | VX: {self.vx:.1f} | VY: {self.vy:.1f} | VZ: {self.vz:.1f} | Yaw: {self.yaw_rate:.2f}"
                print(f"\r{status}", end='', flush=True)

        except AttributeError:
            pass

        

    def publish_twist(self):
        """ REPEATING LOOP (10Hz): Executes the active control mode """
        twist = Twist()

        if self.mode == "MANUAL":
            twist.linear.x = self.vx
            twist.linear.y = self.vy
            twist.linear.z = self.vz
            twist.angular.z = self.yaw_rate
        
        elif self.mode == "AUTO":
            # --- CONTINUOUS AUTO LOGIC (Runs every 0.1s) ---
            
            # A. Altitude Hold PID
            alt_error = self.altitude_setpoint - self.current_altitude
            # self.alt_integral += alt_error # Ensure you have an integral accumulator
            
            # Simple P control (add D if it bounces)
            output_z = self.alt_kp * alt_error 
            self.auto_vz = max(-2.0, min(2.0, output_z))
            self.auto_yaw = -self.kp_pursuit_yaw * self.yaw_error
            self.auto_vx = -self.kp_pursuit_vx * self.vx_error

            # B. Target Pursuit (Inject errors from RedCarDetector)
            # These values should be updated by your /target_errors subscriber
            # self.auto_vx = -self.target_error_vx * self.kp_pursuit_vx
            # self.auto_yaw = -self.target_error_yaw * self.kp_pursuit_yaw

            # Populate Twist
            twist.linear.x = self.auto_vx
            twist.linear.z = self.auto_vz
            twist.angular.z = self.auto_yaw
            
            # Continuous Log
            status = f"AUTO | AltErr: {alt_error:.2f} | VZ: {self.auto_vz:.2f} | yaw error: {self.yaw_error:.2f} | Yaw: {self.auto_yaw:.2f}"
            print(f"\r{status}", end='', flush=True)

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DroneTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.vx, node.vy, node.vz, node.yaw_rate = 0.0, 0.0, 0.0, 0.0
        node.publish_twist()
        rclpy.shutdown()

if __name__ == '__main__':
    main()