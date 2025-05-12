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
        self.declare_parameter('safety_distance', 0.5)  # Distance to start slowing down
        self.declare_parameter('stop_distance', 0.01)    # Distance to completely stop
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
        stop_distance = self.get_parameter('stop_distance').value
        
        # Calculate the best steering angle and speed
        steering_angle, speed = self.calculate_drive_command(
            ranges, max_speed, max_steering, safety_distance, stop_distance)
        
        # Create and publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        
        self.drive_pub.publish(drive_msg)

    def calculate_drive_command(self, ranges, max_speed, max_steering, safety_distance, stop_distance):
        # Define the regions of interest in the LiDAR scan
        # Using 360 degrees of the LiDAR, divided into 5 regions
        num_points = len(ranges)
        points_per_region = num_points // 5
        
        # Define regions (in degrees):
        # Region 0: Far left (288-360 degrees)
        # Region 1: Left (216-288 degrees)
        # Region 2: Front (144-216 degrees)
        # Region 3: Right (72-144 degrees)
        # Region 4: Far right (0-72 degrees)
        
        regions = [
            ranges[0:points_per_region],                    # Far right
            ranges[points_per_region:2*points_per_region],  # Right
            ranges[2*points_per_region:3*points_per_region], # Front
            ranges[3*points_per_region:4*points_per_region], # Left
            ranges[4*points_per_region:]                    # Far left
        ]
        
        # Get minimum distances for each region
        region_mins = [np.min(region) for region in regions]
        
        # Check if we need to stop completely
        if any(d < stop_distance for d in region_mins):
            return 0.0, 0.0  # Stop completely
        
        # Check if we need to slow down and turn
        if any(d < safety_distance for d in region_mins):
            # Find the region with the most space
            best_region = np.argmax(region_mins)
            
            # Calculate steering angle based on the best region
            if best_region == 0:  # Far right
                steering_angle = -max_steering
            elif best_region == 1:  # Right
                steering_angle = -max_steering * 0.7
            elif best_region == 2:  # Front
                steering_angle = 0.0
            elif best_region == 3:  # Left
                steering_angle = max_steering * 0.7
            else:  # Far left
                steering_angle = max_steering
            
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