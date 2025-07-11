#!/usr/bin/env python3

"""
Point Cloud Test Data Publisher for ROS2
This script publishes mock point cloud data to test the 3D visualization dashboard.

Usage:
    python3 pointcloud_test_publisher.py

Make sure you have ROS2 and the rosbridge_suite running:
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
import time
import math
import random

class PointCloudTestPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_test_publisher')
        
        # Create publishers for different point cloud topics
        self.cloud_map_pub = self.create_publisher(PointCloud2, '/cloud_map', 10)
        self.rtabmap_cloud_pub = self.create_publisher(PointCloud2, '/rtabmap/cloud_map', 10)
        self.zed_cloud_pub = self.create_publisher(PointCloud2, '/zed/zed_node/point_cloud/cloud_registered', 10)
        
        # Timer to publish data every 100ms (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_point_cloud)
        
        # Initialize point cloud state
        self.time_offset = time.time()
        self.frame_id = "map"
        self.point_count = 5000  # Number of points to generate
        
        self.get_logger().info('Point Cloud Test Publisher started')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  /cloud_map')
        self.get_logger().info('  /rtabmap/cloud_map')
        self.get_logger().info('  /zed/zed_node/point_cloud/cloud_registered')
        self.get_logger().info(f'Generating {self.point_count} points per cloud')

    def create_point_cloud_message(self, points, colors=None):
        """Create a PointCloud2 message from points and colors"""
        
        # Define the fields for the point cloud
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        point_step = 12  # 3 floats * 4 bytes each
        
        # Add RGB field if colors are provided
        if colors is not None:
            fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
            point_step = 16  # 3 floats + 1 uint32
        
        # Create the message
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * msg.width
        msg.is_dense = True
        
        # Pack the data
        data = []
        for i, point in enumerate(points):
            # Pack x, y, z
            data.extend(struct.pack('fff', point[0], point[1], point[2]))
            
            # Pack RGB if available
            if colors is not None:
                color = colors[i]
                # Convert RGB to packed format (assuming RGB values are 0-255)
                r = int(color[0] * 255) if color[0] <= 1.0 else int(color[0])
                g = int(color[1] * 255) if color[1] <= 1.0 else int(color[1])
                b = int(color[2] * 255) if color[2] <= 1.0 else int(color[2])
                rgb_packed = (r << 16) | (g << 8) | b
                data.extend(struct.pack('I', rgb_packed))
        
        msg.data = bytes(data)
        return msg

    def generate_room_scene(self, current_time):
        """Generate a realistic room scene with furniture and walls"""
        points = []
        colors = []
        
        # Room dimensions
        room_width = 8.0
        room_length = 10.0
        room_height = 3.0
        
        # Generate floor points
        for x in np.linspace(-room_width/2, room_width/2, 50):
            for y in np.linspace(-room_length/2, room_length/2, 60):
                z = 0.0 + random.uniform(-0.02, 0.02)  # Floor with slight noise
                points.append([x, y, z])
                colors.append([0.6, 0.6, 0.6])  # Gray floor
        
        # Generate walls
        # Front and back walls
        for x in np.linspace(-room_width/2, room_width/2, 40):
            for z in np.linspace(0, room_height, 20):
                # Front wall
                y = room_length/2 + random.uniform(-0.02, 0.02)
                points.append([x, y, z])
                colors.append([0.8, 0.8, 0.7])  # Light wall color
                
                # Back wall
                y = -room_length/2 + random.uniform(-0.02, 0.02)
                points.append([x, y, z])
                colors.append([0.8, 0.8, 0.7])
        
        # Side walls
        for y in np.linspace(-room_length/2, room_length/2, 50):
            for z in np.linspace(0, room_height, 20):
                # Left wall
                x = -room_width/2 + random.uniform(-0.02, 0.02)
                points.append([x, y, z])
                colors.append([0.8, 0.8, 0.7])
                
                # Right wall
                x = room_width/2 + random.uniform(-0.02, 0.02)
                points.append([x, y, z])
                colors.append([0.8, 0.8, 0.7])
        
        # Generate furniture
        # Table
        table_x, table_y = 1.0, 0.0
        table_height = 0.8
        for x in np.linspace(table_x - 0.8, table_x + 0.8, 15):
            for y in np.linspace(table_y - 0.4, table_y + 0.4, 8):
                z = table_height + random.uniform(-0.01, 0.01)
                points.append([x, y, z])
                colors.append([0.6, 0.4, 0.2])  # Brown table
        
        # Chair
        chair_x, chair_y = 1.0, -1.2
        chair_height = 0.45
        for x in np.linspace(chair_x - 0.25, chair_x + 0.25, 8):
            for y in np.linspace(chair_y - 0.25, chair_y + 0.25, 8):
                z = chair_height + random.uniform(-0.01, 0.01)
                points.append([x, y, z])
                colors.append([0.4, 0.2, 0.1])  # Dark brown chair
        
        # Add some dynamic elements (moving objects)
        # Simulate a person walking
        person_x = 2.0 * math.sin(current_time * 0.5)
        person_y = 1.0 * math.cos(current_time * 0.3)
        person_height = 1.7
        
        # Person's head/torso points
        for i in range(20):
            x = person_x + random.uniform(-0.2, 0.2)
            y = person_y + random.uniform(-0.2, 0.2)
            z = person_height + random.uniform(-0.3, 0.1)
            points.append([x, y, z])
            colors.append([1.0, 0.8, 0.6])  # Skin color
        
        # Add some random objects/clutter
        for _ in range(100):
            x = random.uniform(-room_width/2 + 0.5, room_width/2 - 0.5)
            y = random.uniform(-room_length/2 + 0.5, room_length/2 - 0.5)
            z = random.uniform(0.1, 2.0)
            points.append([x, y, z])
            # Random colors for objects
            colors.append([random.uniform(0.2, 1.0), random.uniform(0.2, 1.0), random.uniform(0.2, 1.0)])
        
        return points, colors

    def generate_outdoor_scene(self, current_time):
        """Generate an outdoor scene with terrain and objects"""
        points = []
        colors = []
        
        # Generate terrain
        terrain_size = 20.0
        for x in np.linspace(-terrain_size/2, terrain_size/2, 80):
            for y in np.linspace(-terrain_size/2, terrain_size/2, 80):
                # Create rolling hills
                z = (math.sin(x * 0.3) * math.cos(y * 0.2) * 2.0 + 
                     math.sin(x * 0.1) * 1.0 + 
                     random.uniform(-0.1, 0.1))
                points.append([x, y, z])
                
                # Color based on height (green for low, brown for high)
                height_factor = (z + 3) / 6  # Normalize to 0-1
                green_intensity = max(0.2, 1.0 - height_factor)
                brown_intensity = max(0.1, height_factor)
                colors.append([brown_intensity * 0.6, green_intensity, brown_intensity * 0.3])
        
        # Add trees
        tree_positions = [(-5, -3), (3, -7), (-8, 5), (6, 8), (0, 0)]
        for tree_x, tree_y in tree_positions:
            # Tree trunk
            for z in np.linspace(0, 3, 15):
                x = tree_x + random.uniform(-0.1, 0.1)
                y = tree_y + random.uniform(-0.1, 0.1)
                points.append([x, y, z])
                colors.append([0.4, 0.2, 0.1])  # Brown trunk
            
            # Tree foliage
            for _ in range(50):
                x = tree_x + random.uniform(-1.5, 1.5)
                y = tree_y + random.uniform(-1.5, 1.5)
                z = 2.5 + random.uniform(0, 1.5)
                points.append([x, y, z])
                colors.append([0.1, random.uniform(0.4, 0.8), 0.1])  # Green leaves
        
        # Add a moving vehicle
        vehicle_x = 8.0 * math.sin(current_time * 0.2)
        vehicle_y = 5.0
        vehicle_z = 1.0
        
        for i in range(30):
            x = vehicle_x + random.uniform(-1.0, 1.0)
            y = vehicle_y + random.uniform(-0.5, 0.5)
            z = vehicle_z + random.uniform(-0.2, 0.5)
            points.append([x, y, z])
            colors.append([0.8, 0.1, 0.1])  # Red vehicle
        
        return points, colors

    def publish_point_cloud(self):
        current_time = time.time() - self.time_offset
        
        # Alternate between indoor and outdoor scenes
        if int(current_time / 30) % 2 == 0:  # Switch every 30 seconds
            points, colors = self.generate_room_scene(current_time)
            scene_type = "Indoor Room"
        else:
            points, colors = self.generate_outdoor_scene(current_time)
            scene_type = "Outdoor Terrain"
        
        # Limit the number of points
        if len(points) > self.point_count:
            indices = np.random.choice(len(points), self.point_count, replace=False)
            points = [points[i] for i in indices]
            colors = [colors[i] for i in indices]
        
        # Create point cloud message
        cloud_msg = self.create_point_cloud_message(points, colors)
        
        # Publish to all topics
        self.cloud_map_pub.publish(cloud_msg)
        self.rtabmap_cloud_pub.publish(cloud_msg)
        self.zed_cloud_pub.publish(cloud_msg)
        
        # Log occasionally
        if int(current_time * 10) % 50 == 0:  # Every 5 seconds
            self.get_logger().info(
                f'Published {len(points)} points - Scene: {scene_type} - '
                f'Time: {current_time:.1f}s'
            )

def main(args=None):
    rclpy.init(args=args)
    
    pointcloud_publisher = PointCloudTestPublisher()
    
    try:
        rclpy.spin(pointcloud_publisher)
    except KeyboardInterrupt:
        pointcloud_publisher.get_logger().info('Point Cloud Test Publisher stopped by user')
    finally:
        pointcloud_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
