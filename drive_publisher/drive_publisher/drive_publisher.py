#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math

class DrivePublisher(Node):
    def __init__(self):
        super().__init__('drive_publisher')
        
        # Create subscribers and publishers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            'drive',
            10)
        
        # Parameters
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('max_steering_angle', 0.34)
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('angle_increment', 0.0058)  # LiDAR angle increment
        
        self.get_logger().info('Drive Publisher Node has been started')

    def scan_callback(self, scan_msg):
        # Convert scan data to numpy array for easier processing
        ranges = np.array(scan_msg.ranges)
        
        # Replace inf values with a large number
        ranges[np.isinf(ranges)] = 100.0
        
        # Get parameters
        max_speed = self.get_parameter('max_speed').value
        max_steering = self.get_parameter('max_steering_angle').value
        safety_distance = self.get_parameter('safety_distance').value
        
        # Calculate the best steering angle and speed
        steering_angle, speed = self.calculate_drive_command(ranges, max_speed, max_steering, safety_distance)
        
        # Create and publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        
        self.drive_pub.publish(drive_msg)

    def calculate_drive_command(self, ranges, max_speed, max_steering, safety_distance):
        # Define the regions of interest in the LiDAR scan
        # Front region (slightly wider than the car)
        front_region = ranges[300:420]  # Approximately ±30 degrees in front
        
        # Check if there are obstacles in the front region
        if np.min(front_region) < safety_distance:
            # Obstacle detected, need to turn
            # Find the direction with the most space
            left_region = ranges[420:540]  # Left side
            right_region = ranges[180:300]  # Right side
            
            left_min = np.min(left_region)
            right_min = np.min(right_region)
            
            # Turn towards the side with more space
            if left_min > right_min:
                steering_angle = max_steering  # Turn left
            else:
                steering_angle = -max_steering  # Turn right
            
            # Reduce speed when obstacle is close
            speed = max_speed * 0.5
        else:
            # No immediate obstacles, can drive straight
            steering_angle = 0.0
            speed = max_speed
        
        return steering_angle, speed

def main(args=None):
    rclpy.init(args=args)
    drive_publisher = DrivePublisher()
    rclpy.spin(drive_publisher)
    drive_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 