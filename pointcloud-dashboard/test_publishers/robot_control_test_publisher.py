#!/usr/bin/env python3

"""
Robot Control Test Data Publisher for ROS2
This script publishes mock robot control data and listens for control commands
to test the robot control dashboard and joystick functionality.

Usage:
    python3 robot_control_test_publisher.py

Make sure you have ROS2 and the rosbridge_suite running:
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from sensor_msgs.msg import Joy
import json
import math
import time
import random

class RobotControlTestPublisher(Node):
    def __init__(self):
        super().__init__('robot_control_test_publisher')
        
        # Publishers for robot status and feedback
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.robot_status_pub = self.create_publisher(String, '/robot_status', 10)
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        
        # Subscribers for control commands
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        
        # Timer to publish robot status every 100ms (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_robot_data)
        
        # Robot state
        self.time_offset = time.time()
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.last_cmd_time = 0.0
        
        # Navigation state
        self.current_goal = None
        self.navigation_status = "idle"
        self.distance_to_goal = 0.0
        
        # Joystick simulation
        self.simulate_joystick = True
        self.joystick_timer = self.create_timer(0.05, self.publish_joystick_data)  # 20 Hz
        
        self.get_logger().info('Robot Control Test Publisher started')
        self.get_logger().info('Publishing robot status and odometry data')
        self.get_logger().info('Listening for control commands on:')
        self.get_logger().info('  /cmd_vel (Twist)')
        self.get_logger().info('  /move_base_simple/goal (PoseStamped)')
        self.get_logger().info('  /initialpose (PoseWithCovarianceStamped)')

    def cmd_vel_callback(self, msg):
        """Handle velocity commands from joystick or other controllers"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        self.last_cmd_time = time.time()
        
        self.get_logger().info(
            f'Received cmd_vel: linear={self.linear_vel:.2f} m/s, '
            f'angular={self.angular_vel:.2f} rad/s'
        )

    def goal_callback(self, msg):
        """Handle navigation goal commands"""
        self.current_goal = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'theta': 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        }
        self.navigation_status = "navigating"
        
        self.get_logger().info(
            f'Received navigation goal: x={self.current_goal["x"]:.2f}, '
            f'y={self.current_goal["y"]:.2f}, theta={self.current_goal["theta"]:.2f}'
        )

    def initial_pose_callback(self, msg):
        """Handle initial pose setting"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_theta = 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        
        self.get_logger().info(
            f'Robot pose initialized: x={self.robot_x:.2f}, '
            f'y={self.robot_y:.2f}, theta={self.robot_theta:.2f}'
        )

    def update_robot_position(self, dt):
        """Update robot position based on current velocities"""
        # Simple kinematic model
        if time.time() - self.last_cmd_time < 0.5:  # Commands are recent
            # Update position
            self.robot_x += self.linear_vel * math.cos(self.robot_theta) * dt
            self.robot_y += self.linear_vel * math.sin(self.robot_theta) * dt
            self.robot_theta += self.angular_vel * dt
            
            # Normalize theta
            while self.robot_theta > math.pi:
                self.robot_theta -= 2 * math.pi
            while self.robot_theta < -math.pi:
                self.robot_theta += 2 * math.pi
        else:
            # No recent commands, robot stops
            self.linear_vel *= 0.9  # Gradual deceleration
            self.angular_vel *= 0.9
            
            if abs(self.linear_vel) < 0.01:
                self.linear_vel = 0.0
            if abs(self.angular_vel) < 0.01:
                self.angular_vel = 0.0

    def update_navigation_status(self):
        """Update navigation status based on current goal"""
        if self.current_goal is None:
            self.navigation_status = "idle"
            self.distance_to_goal = 0.0
            return
        
        # Calculate distance to goal
        dx = self.current_goal['x'] - self.robot_x
        dy = self.current_goal['y'] - self.robot_y
        self.distance_to_goal = math.sqrt(dx*dx + dy*dy)
        
        # Check if goal is reached
        if self.distance_to_goal < 0.2:  # 20cm tolerance
            self.navigation_status = "goal_reached"
            self.current_goal = None
        elif self.distance_to_goal < 1.0:
            self.navigation_status = "approaching_goal"
        else:
            self.navigation_status = "navigating"

    def publish_robot_data(self):
        current_time = time.time() - self.time_offset
        dt = 0.1  # 100ms
        
        # Update robot state
        self.update_robot_position(dt)
        self.update_navigation_status()
        
        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # Position
        odom_msg.pose.pose.position.x = self.robot_x
        odom_msg.pose.pose.position.y = self.robot_y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from theta)
        odom_msg.pose.pose.orientation.z = math.sin(self.robot_theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.robot_theta / 2)
        
        # Velocity
        odom_msg.twist.twist.linear.x = self.linear_vel
        odom_msg.twist.twist.angular.z = self.angular_vel
        
        # Add some covariance (uncertainty)
        odom_msg.pose.covariance[0] = 0.1  # x
        odom_msg.pose.covariance[7] = 0.1  # y
        odom_msg.pose.covariance[35] = 0.1  # theta
        
        self.odom_pub.publish(odom_msg)
        
        # Publish robot status
        status_data = {
            "timestamp": current_time,
            "position": {
                "x": self.robot_x,
                "y": self.robot_y,
                "theta": self.robot_theta
            },
            "velocity": {
                "linear": self.linear_vel,
                "angular": self.angular_vel
            },
            "navigation": {
                "status": self.navigation_status,
                "distance_to_goal": self.distance_to_goal,
                "current_goal": self.current_goal
            },
            "system": {
                "motors_enabled": True,
                "emergency_stop": False,
                "battery_level": 85.0 + 10 * math.sin(current_time * 0.01),  # Slowly varying
                "connection_quality": random.uniform(0.8, 1.0),
                "localization_quality": random.uniform(0.7, 1.0)
            },
            "sensors": {
                "lidar_active": True,
                "camera_active": True,
                "imu_active": True,
                "gps_active": random.choice([True, False])  # Simulate GPS availability
            }
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.robot_status_pub.publish(status_msg)
        
        # Log occasionally
        if int(current_time * 10) % 50 == 0:  # Every 5 seconds
            self.get_logger().info(
                f'Robot Status - Pos: ({self.robot_x:.2f}, {self.robot_y:.2f}, {math.degrees(self.robot_theta):.1f}°), '
                f'Vel: ({self.linear_vel:.2f}, {self.angular_vel:.2f}), '
                f'Nav: {self.navigation_status}'
            )

    def publish_joystick_data(self):
        """Simulate joystick input for testing"""
        if not self.simulate_joystick:
            return
        
        current_time = time.time() - self.time_offset
        
        # Create a Joy message
        joy_msg = Joy()
        joy_msg.header = Header()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = "joystick"
        
        # Simulate joystick axes (8 axes typical for many joysticks)
        # Axes 0,1: Left stick (X, Y)
        # Axes 2,3: Right stick (X, Y)  
        # Axes 4,5: Triggers (L2, R2)
        # Axes 6,7: D-pad (X, Y)
        
        # Simulate some movement patterns
        left_stick_x = 0.3 * math.sin(current_time * 0.5)  # Gentle side-to-side
        left_stick_y = 0.2 * math.cos(current_time * 0.3)  # Gentle forward-back
        
        joy_msg.axes = [
            left_stick_x,    # Left stick X (steering)
            left_stick_y,    # Left stick Y (throttle)
            0.0,             # Right stick X
            0.0,             # Right stick Y
            0.0,             # Left trigger
            0.0,             # Right trigger
            0.0,             # D-pad X
            0.0              # D-pad Y
        ]
        
        # Simulate buttons (12 buttons typical)
        # Randomly press some buttons occasionally
        buttons = [False] * 12
        if random.random() < 0.05:  # 5% chance per update
            button_index = random.randint(0, 11)
            buttons[button_index] = True
        
        joy_msg.buttons = [int(b) for b in buttons]
        
        self.joy_pub.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)
    
    robot_control_publisher = RobotControlTestPublisher()
    
    try:
        rclpy.spin(robot_control_publisher)
    except KeyboardInterrupt:
        robot_control_publisher.get_logger().info('Robot Control Test Publisher stopped by user')
    finally:
        robot_control_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
