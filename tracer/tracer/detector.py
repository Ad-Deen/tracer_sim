#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Vector3  
# Using x: vx_err, y: yaw_err, z: target_found

class RedCarDetector(Node):
    def __init__(self):
        super().__init__('red_car_detector')
        self.subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.bridge = CvBridge()
        
        #re aqusition variables
        self.last_seen_time = 0.0
        self.target_lost_timeout = 1.0  
        # Seconds to keep z=1 after losing target
        self.has_had_first_lock = False 
        # New flag to prevent false-starts
        self.predictive_mult = 2.0
        self.vx_error = 0.0
        self.yaw_error = 0.0
        self.detected = False
        self.w_img = 320
        self.h = 240
        # --- NEW: Define Trapezoid (20% central zone) ---
        # Perspective-compensated: bottom is wider than top
        self.trap_pts = np.array([
            [int(self.w_img*0.42), int(self.h*0.40)], # Top Left
            [int(self.w_img*0.58), int(self.h*0.40)], # Top Right
            [int(self.w_img*0.65), int(self.h*0.60)], # Bottom Right
            [int(self.w_img*0.35), int(self.h*0.60)]  # Bottom Left
        ], np.int32)

        self.error_publisher = self.create_publisher(Vector3, '/target_errors', 10)
        self.get_logger().info("Red Car Detector Node Started")

    def image_callback(self, msg):
        # 1. Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # h, w_img = frame.shape[:2] # 240, 320

        current_time = self.get_clock().now().nanoseconds / 1e9 # Current time in seconds

        # Default message (no target found)
        error_msg = Vector3()        
        
        
        # Draw the Trapezoid for visual feedback
        cv2.polylines(frame, [self.trap_pts], True, (255, 255, 0), 2)
        
        # Optional: Apply a slight Gaussian Blur
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 2. ULTRA-WIDE RANGES
        lower_red1, upper_red1 = np.array([0, 40, 40]), np.array([10, 255, 255])
        lower_red2, upper_red2 = np.array([160, 40, 40]), np.array([180, 255, 255])
        mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), 
                              cv2.inRange(hsv, lower_red2, upper_red2))

        # 3. ROBUST MORPHOLOGY
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        # 4. DYNAMIC DISTANCE DETECTION
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        actual_detection = False
        for cnt in contours:
            
            area = cv2.contourArea(cnt)
            if area > 10:
                # ... valid detection confirmed ...
                actual_detection = True
                self.has_had_first_lock = True
                self.last_seen_time = current_time  
                x, y, w, h_box = cv2.boundingRect(cnt)
                aspect_ratio = float(w) / h_box
                
                if 0.4 < aspect_ratio < 4.0:
                    actual_detection = True
                    cx, cy = x + w//2, y + h_box//2
                    
                    # --- NEW: Error Calculation ---
                    # Normalize errors to [-1, 1] relative to image center
                    # Yaw error (Horizontal): Right is positive, Left is negative
                    self.yaw_error = ((cx - (self.w_img / 2)) / (self.w_img / 2)) * self.predictive_mult
                    # Vx error (Vertical): Bottom (Close) is positive, Top (Far) is negative
                    self.vx_error = ((cy - (self.h / 2)) / (self.h / 2)) * self.predictive_mult
                    
                    # Check if center is in Trapezoid
                    is_safe = cv2.pointPolygonTest(self.trap_pts, (float(cx), float(cy)), False)
                    
                    # Visual Feedback
                    color = (0, 255, 0) if is_safe >= 0 else (0, 0, 255)
                    cv2.rectangle(frame, (x, y), (x + w, y + h_box), color, 2)
                    cv2.circle(frame, (cx, cy), 5, color, -1)
                    
                    # # Visual display of raw errors
                    # txt = f"Vx Err: {vx_error:.2f} | Yaw Err: {yaw_error:.2f}"
                    # cv2.putText(frame, txt, (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    if is_safe >= 0:
                        cv2.putText(frame, "STATUS: SAFE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        # Inside safe zone: errors are zeroed
                        error_msg.x = 0.0
                        error_msg.y = 0.0
                        error_msg.z = 1.0  # target_found = True
                    else:
                        cv2.putText(frame, "STATUS: TRACKING", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        error_msg.x = self.vx_error
                        error_msg.y = self.yaw_error
                        error_msg.z = 1.0  # target_found = True
                    
                    
                    
                    # Break after first valid car to avoid multi-target jitter
                    # break 
        # --- UPDATED COASTING LOGIC ---
        if not actual_detection:
            # Only "coast" if we've seen the car at least once since node started
            if self.has_had_first_lock:
                time_since_last_seen = current_time - self.last_seen_time
                
                if time_since_last_seen < self.target_lost_timeout:
                    error_msg.z = 1.0  # Keep target_found = True
                    # Keep errors at 0.0 so drone stops/hovers during search
                    error_msg.x = self.vx_error
                    error_msg.y = self.yaw_error
                    # self.get_logger().info(f"Coasting... {time_since_last_seen:.2f}s")
                else:
                    error_msg.z = 0.0
                    error_msg.x = 0.0
                    error_msg.y = 0.0
                    # self.get_logger().info("Target truly lost.")
            else:
                # Haven't seen the car yet, keep z=0
                error_msg.z = 0.0

        self.error_publisher.publish(error_msg)

        # 5. Display the result
        cv2.imshow("Drone Vision - Distance Adaptive", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = RedCarDetector()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()