#!/usr/bin/env python3

"""
IMU Test Data Publisher for ROS2
This script publishes mock IMU data to test the IMU sensor dashboard.

Usage:
    python3 imu_test_publisher.py

Make sure you have ROS2 and the rosbridge_suite running:
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
import math
import time

class IMUTestPublisher(Node):
    def __init__(self):
        super().__init__('imu_test_publisher')
        
        # Create publisher for IMU JSON data
        self.imu_pub = self.create_publisher(String, '/perception_brain/imu/json', 10)
        
        # Timer to publish data every 50ms (20 Hz)
        self.timer = self.create_timer(0.05, self.publish_imu_data)
        
        # Initialize IMU state
        self.time_offset = time.time()
        
        self.get_logger().info('IMU Test Publisher started. Publishing to /perception_brain/imu/json')
        self.get_logger().info('Publishing at 20 Hz for smooth visualization')

    def publish_imu_data(self):
        # Generate realistic IMU data with motion patterns
        current_time = time.time() - self.time_offset
        
        # Simulate some realistic motion patterns
        roll = math.sin(current_time * 0.1) * 15 + random.uniform(-2, 2)  # ±15 degrees with noise
        pitch = math.cos(current_time * 0.15) * 10 + random.uniform(-1.5, 1.5)  # ±10 degrees with noise
        yaw = (current_time * 5) % 360  # Continuous rotation
        
        # Convert Euler to quaternion for realistic data
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)
        
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)

        # Quaternion calculation
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        # Generate realistic sensor data
        imu_data = {
            "timestamp": current_time,
            "valid": True,
            "orientation": {
                "w": qw,
                "x": qx,
                "y": qy,
                "z": qz
            },
            "gyroscope": {
                "x": (random.random() - 0.5) * 0.2 + math.sin(current_time * 0.3) * 0.1,  # rad/s
                "y": (random.random() - 0.5) * 0.2 + math.cos(current_time * 0.4) * 0.1,
                "z": (random.random() - 0.5) * 0.4 + math.sin(current_time * 0.2) * 0.05
            },
            "accelerometer": {
                "x": (random.random() - 0.5) * 2 + math.sin(current_time * 0.5) * 0.5,  # m/s²
                "y": (random.random() - 0.5) * 2 + math.cos(current_time * 0.6) * 0.5,
                "z": 9.81 + (random.random() - 0.5) * 0.5  # Gravity ± noise
            },
            "magnetometer": {
                "x": -1.25 + (random.random() - 0.5) * 0.1,  # μT
                "y": 0.0625 + (random.random() - 0.5) * 0.05,
                "z": -5 + (random.random() - 0.5) * 0.2
            },
            "gravity": {
                "x": 0 + (random.random() - 0.5) * 0.1,
                "y": 0 + (random.random() - 0.5) * 0.1,
                "z": 9.81 + (random.random() - 0.5) * 0.1
            },
            "temperature": 35 + math.sin(current_time * 0.01) * 5 + random.uniform(-1, 1),  # °C
            "calibration_status": {
                "system": random.choice([0, 1, 2, 3]),
                "gyroscope": random.choice([0, 1, 2, 3]),
                "accelerometer": random.choice([0, 1, 2, 3]),
                "magnetometer": random.choice([0, 1, 2, 3])
            }
        }
        
        # Create and publish message
        msg = String()
        msg.data = json.dumps(imu_data)
        self.imu_pub.publish(msg)
        
        # Log current values occasionally
        if int(current_time * 20) % 40 == 0:  # Every 2 seconds
            self.get_logger().info(
                f'Published IMU: Roll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°, '
                f'Temp={imu_data["temperature"]:.1f}°C'
            )

def main(args=None):
    rclpy.init(args=args)
    
    imu_publisher = IMUTestPublisher()
    
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        imu_publisher.get_logger().info('IMU Test Publisher stopped by user')
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
