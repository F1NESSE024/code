#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math

class AutonomousDriver(Node):
    def __init__(self):
        super().__init__('autonomous_driver')
        
        # Create subscribers and publishers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)
            
        # Parameters
        self.declare_parameter('max_speed', 2.0)  # Maximum speed in m/s
        self.declare_parameter('max_steering_angle', 0.34)  # Maximum steering angle in radians
        self.declare_parameter('safety_distance', 0.5)  # Minimum distance to obstacles in meters
        self.declare_parameter('lookahead_distance', 1.0)  # Distance to look ahead for path planning
        
        # Get parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        
        self.get_logger().info('Autonomous driver node initialized')
        
    def scan_callback(self, scan_msg):
        """
        Process LiDAR scan data and generate drive commands
        """
        # Convert scan ranges to numpy array
        ranges = np.array(scan_msg.ranges)
        
        # Replace inf values with max range
        ranges[np.isinf(ranges)] = scan_msg.range_max
        
        # Get the angle increment
        angle_increment = scan_msg.angle_increment
        
        # Calculate angles for each range measurement
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max + angle_increment, angle_increment)
        
        # Find the best path
        speed, steering_angle = self.find_best_path(ranges, angles)
        
        # Create and publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        
        self.drive_pub.publish(drive_msg)
        
    def find_best_path(self, ranges, angles):
        """
        Find the best path based on LiDAR data
        Returns speed and steering angle
        """
        # Find the widest gap in front of the car
        front_indices = np.where(np.abs(angles) < math.pi/4)[0]  # Look at ±45 degrees in front
        front_ranges = ranges[front_indices]
        
        # Check if there are obstacles too close
        if np.min(front_ranges) < self.safety_distance:
            # Emergency stop
            return 0.0, 0.0
            
        # Find the widest gap
        gap_start = None
        gap_end = None
        max_gap_width = 0
        current_gap_start = None
        
        for i in range(len(front_ranges)):
            if front_ranges[i] > self.lookahead_distance:
                if current_gap_start is None:
                    current_gap_start = i
            else:
                if current_gap_start is not None:
                    gap_width = i - current_gap_start
                    if gap_width > max_gap_width:
                        max_gap_width = gap_width
                        gap_start = current_gap_start
                        gap_end = i
                    current_gap_start = None
                    
        # If we found a gap, steer towards its center
        if gap_start is not None and gap_end is not None:
            gap_center = (gap_start + gap_end) / 2
            center_angle = angles[front_indices[gap_center]]
            
            # Calculate steering angle (proportional to the angle to the gap center)
            steering_angle = np.clip(center_angle, -self.max_steering_angle, self.max_steering_angle)
            
            # Calculate speed based on how wide the gap is
            gap_width_angle = abs(angles[front_indices[gap_end]] - angles[front_indices[gap_start]])
            speed_factor = min(1.0, gap_width_angle / (math.pi/2))  # Normalize to [0,1]
            speed = self.max_speed * speed_factor
            
            return speed, steering_angle
            
        # If no good gap found, slow down and search
        return self.max_speed * 0.5, 0.0

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 