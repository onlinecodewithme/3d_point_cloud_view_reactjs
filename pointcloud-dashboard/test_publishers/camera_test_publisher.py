#!/usr/bin/env python3

"""
Camera Feed Test Data Publisher for ROS2
This script publishes mock camera image data to test the camera feed viewer.

Usage:
    python3 camera_test_publisher.py

Make sure you have ROS2 and the rosbridge_suite running:
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
import numpy as np
import cv2
import time
import math
import random
from cv_bridge import CvBridge

class CameraTestPublisher(Node):
    def __init__(self):
        super().__init__('camera_test_publisher')
        
        # Create publishers for different camera topics
        self.zed_left_pub = self.create_publisher(Image, '/zed/zed_node/left/image_rect_color', 10)
        self.zed_right_pub = self.create_publisher(Image, '/zed/zed_node/right/image_rect_color', 10)
        self.zed_depth_pub = self.create_publisher(Image, '/zed/zed_node/depth/depth_registered', 10)
        self.camera_raw_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.camera_compressed_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Timer to publish data every 33ms (~30 FPS)
        self.timer = self.create_timer(0.033, self.publish_camera_data)
        
        # Initialize camera state
        self.time_offset = time.time()
        self.frame_count = 0
        self.image_width = 640
        self.image_height = 480
        
        self.get_logger().info('Camera Test Publisher started')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  /zed/zed_node/left/image_rect_color')
        self.get_logger().info('  /zed/zed_node/right/image_rect_color')
        self.get_logger().info('  /zed/zed_node/depth/depth_registered')
        self.get_logger().info('  /camera/image_raw')
        self.get_logger().info('  /camera/image_raw/compressed')
        self.get_logger().info(f'Resolution: {self.image_width}x{self.image_height} @ ~30 FPS')

    def create_synthetic_scene(self, current_time, camera_type="left"):
        """Create a synthetic scene with moving objects"""
        # Create base image
        img = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        
        # Background gradient (sky to ground)
        for y in range(self.image_height):
            intensity = int(255 * (1 - y / self.image_height))
            sky_color = max(100, intensity)
            ground_color = max(50, 255 - intensity)
            img[y, :] = [sky_color, sky_color, min(255, sky_color + 50)]  # Blueish sky
        
        # Add ground
        ground_start = int(self.image_height * 0.6)
        img[ground_start:, :] = [34, 139, 34]  # Green ground
        
        # Add some buildings/obstacles
        buildings = [
            (50, ground_start - 100, 80, 100),   # (x, y, width, height)
            (200, ground_start - 80, 60, 80),
            (350, ground_start - 120, 100, 120),
            (500, ground_start - 60, 90, 60)
        ]
        
        for bx, by, bw, bh in buildings:
            if bx + bw < self.image_width and by >= 0:
                # Building body
                cv2.rectangle(img, (bx, by), (bx + bw, by + bh), (100, 100, 100), -1)
                # Building outline
                cv2.rectangle(img, (bx, by), (bx + bw, by + bh), (80, 80, 80), 2)
                
                # Add windows
                for wx in range(bx + 10, bx + bw - 10, 20):
                    for wy in range(by + 10, by + bh - 10, 25):
                        if wx + 8 < bx + bw and wy + 12 < by + bh:
                            # Random window light
                            if random.random() > 0.5:
                                cv2.rectangle(img, (wx, wy), (wx + 8, wy + 12), (255, 255, 150), -1)
                            else:
                                cv2.rectangle(img, (wx, wy), (wx + 8, wy + 12), (50, 50, 100), -1)
        
        # Add moving objects
        # Moving car
        car_x = int(100 + 200 * math.sin(current_time * 0.5))
        car_y = ground_start - 30
        if 0 <= car_x <= self.image_width - 60:
            cv2.rectangle(img, (car_x, car_y), (car_x + 60, car_y + 25), (200, 50, 50), -1)  # Car body
            cv2.circle(img, (car_x + 10, car_y + 25), 8, (0, 0, 0), -1)  # Wheel 1
            cv2.circle(img, (car_x + 50, car_y + 25), 8, (0, 0, 0), -1)  # Wheel 2
        
        # Moving person
        person_x = int(300 + 150 * math.cos(current_time * 0.3))
        person_y = ground_start - 50
        if 0 <= person_x <= self.image_width - 20:
            cv2.rectangle(img, (person_x, person_y), (person_x + 15, person_y + 40), (255, 200, 150), -1)  # Body
            cv2.circle(img, (person_x + 7, person_y - 10), 8, (255, 220, 177), -1)  # Head
        
        # Add some dynamic elements
        # Floating particles (dust, leaves, etc.)
        for _ in range(20):
            px = int(random.uniform(0, self.image_width))
            py = int(random.uniform(0, self.image_height))
            size = random.randint(1, 3)
            color = (random.randint(200, 255), random.randint(200, 255), random.randint(150, 255))
            cv2.circle(img, (px, py), size, color, -1)
        
        # Add timestamp overlay
        timestamp_text = f"Frame: {self.frame_count} | Time: {current_time:.1f}s | {camera_type.upper()}"
        cv2.putText(img, timestamp_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(img, timestamp_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        
        # Add camera type specific modifications
        if camera_type == "right":
            # Simulate stereo offset by shifting image slightly
            M = np.float32([[1, 0, -10], [0, 1, 0]])  # Shift left by 10 pixels
            img = cv2.warpAffine(img, M, (self.image_width, self.image_height))
            
        elif camera_type == "depth":
            # Convert to depth-like visualization
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Simulate depth based on vertical position (closer objects at bottom)
            depth_img = np.zeros_like(img)
            for y in range(self.image_height):
                depth_value = int(255 * (y / self.image_height))  # Closer = higher value
                depth_img[y, :] = [depth_value, depth_value, depth_value]
            
            # Blend with original scene structure
            img = cv2.addWeighted(depth_img, 0.7, img, 0.3, 0)
        
        return img

    def create_indoor_scene(self, current_time):
        """Create an indoor scene"""
        img = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        
        # Room background
        img[:] = [180, 180, 160]  # Light gray walls
        
        # Floor
        floor_start = int(self.image_height * 0.7)
        img[floor_start:, :] = [139, 69, 19]  # Brown floor
        
        # Add furniture
        # Table
        table_x, table_y = 200, floor_start - 60
        cv2.rectangle(img, (table_x, table_y), (table_x + 120, table_y + 60), (101, 67, 33), -1)
        
        # Chair
        chair_x, chair_y = 150, floor_start - 40
        cv2.rectangle(img, (chair_x, chair_y), (chair_x + 40, chair_y + 40), (139, 69, 19), -1)
        
        # Moving robot/object
        robot_x = int(100 + 100 * math.sin(current_time * 0.4))
        robot_y = floor_start - 30
        if 0 <= robot_x <= self.image_width - 40:
            cv2.rectangle(img, (robot_x, robot_y), (robot_x + 40, robot_y + 30), (100, 100, 200), -1)
            cv2.circle(img, (robot_x + 20, robot_y + 15), 5, (255, 0, 0), -1)  # LED indicator
        
        # Add timestamp
        timestamp_text = f"Indoor Scene | Frame: {self.frame_count} | Time: {current_time:.1f}s"
        cv2.putText(img, timestamp_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(img, timestamp_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        return img

    def publish_camera_data(self):
        current_time = time.time() - self.time_offset
        self.frame_count += 1
        
        # Alternate between outdoor and indoor scenes
        if int(current_time / 30) % 2 == 0:  # Switch every 30 seconds
            left_img = self.create_synthetic_scene(current_time, "left")
            right_img = self.create_synthetic_scene(current_time, "right")
            depth_img = self.create_synthetic_scene(current_time, "depth")
            scene_type = "Outdoor"
        else:
            left_img = self.create_indoor_scene(current_time)
            right_img = self.create_indoor_scene(current_time)
            depth_img = cv2.cvtColor(self.create_indoor_scene(current_time), cv2.COLOR_BGR2GRAY)
            depth_img = cv2.cvtColor(depth_img, cv2.COLOR_GRAY2BGR)  # Convert back to 3-channel
            scene_type = "Indoor"
        
        # Create ROS Image messages
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link"
        
        try:
            # Left camera
            left_msg = self.bridge.cv2_to_imgmsg(left_img, "bgr8")
            left_msg.header = header
            self.zed_left_pub.publish(left_msg)
            
            # Right camera
            right_msg = self.bridge.cv2_to_imgmsg(right_img, "bgr8")
            right_msg.header = header
            self.zed_right_pub.publish(right_msg)
            
            # Depth camera
            depth_msg = self.bridge.cv2_to_imgmsg(depth_img, "bgr8")
            depth_msg.header = header
            self.zed_depth_pub.publish(depth_msg)
            
            # Raw camera (use left image)
            raw_msg = self.bridge.cv2_to_imgmsg(left_img, "bgr8")
            raw_msg.header = header
            self.camera_raw_pub.publish(raw_msg)
            
            # Compressed image
            compressed_msg = CompressedImage()
            compressed_msg.header = header
            compressed_msg.format = "jpeg"
            
            # Encode as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            _, compressed_data = cv2.imencode('.jpg', left_img, encode_param)
            compressed_msg.data = compressed_data.tobytes()
            self.camera_compressed_pub.publish(compressed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing camera data: {e}')
        
        # Log occasionally
        if self.frame_count % 90 == 0:  # Every 3 seconds at 30 FPS
            self.get_logger().info(
                f'Published {scene_type} scene - Frame: {self.frame_count}, '
                f'Time: {current_time:.1f}s, FPS: ~30'
            )

def main(args=None):
    rclpy.init(args=args)
    
    camera_publisher = CameraTestPublisher()
    
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        camera_publisher.get_logger().info('Camera Test Publisher stopped by user')
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
