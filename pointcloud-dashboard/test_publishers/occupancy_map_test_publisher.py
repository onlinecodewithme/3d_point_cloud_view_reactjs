#!/usr/bin/env python3

"""
Occupancy Map Test Data Publisher for ROS2
This script publishes mock occupancy grid data to test the navigation and mapping features.

Usage:
    python3 occupancy_map_test_publisher.py

Make sure you have ROS2 and the rosbridge_suite running:
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import numpy as np
import math
import time
import random

class OccupancyMapTestPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_map_test_publisher')
        
        # Create publisher for occupancy map
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Timer to publish data every 5 seconds (maps don't change frequently)
        self.timer = self.create_timer(5.0, self.publish_occupancy_map)
        
        # Map parameters
        self.map_width = 200  # cells
        self.map_height = 200  # cells
        self.resolution = 0.1  # meters per cell
        self.origin_x = -10.0  # meters
        self.origin_y = -10.0  # meters
        
        # Initialize map state
        self.time_offset = time.time()
        self.map_data = np.full((self.map_height, self.map_width), -1, dtype=np.int8)  # Unknown initially
        
        self.get_logger().info('Occupancy Map Test Publisher started')
        self.get_logger().info(f'Publishing to /map')
        self.get_logger().info(f'Map size: {self.map_width}x{self.map_height} cells')
        self.get_logger().info(f'Resolution: {self.resolution} m/cell')
        self.get_logger().info(f'Origin: ({self.origin_x}, {self.origin_y})')

    def create_room_map(self):
        """Create a realistic indoor room map"""
        # Initialize as free space
        map_data = np.full((self.map_height, self.map_width), 0, dtype=np.int8)
        
        # Room dimensions in cells
        room_width = int(8.0 / self.resolution)  # 8 meters
        room_height = int(10.0 / self.resolution)  # 10 meters
        
        # Center the room in the map
        start_x = (self.map_width - room_width) // 2
        start_y = (self.map_height - room_height) // 2
        
        # Create walls (occupied = 100)
        # Top and bottom walls
        map_data[start_y:start_y+5, start_x:start_x+room_width] = 100  # Top wall
        map_data[start_y+room_height-5:start_y+room_height, start_x:start_x+room_width] = 100  # Bottom wall
        
        # Left and right walls
        map_data[start_y:start_y+room_height, start_x:start_x+5] = 100  # Left wall
        map_data[start_y:start_y+room_height, start_x+room_width-5:start_x+room_width] = 100  # Right wall
        
        # Add door opening in one wall
        door_start = start_y + room_height // 2 - 10
        door_end = start_y + room_height // 2 + 10
        map_data[door_start:door_end, start_x:start_x+5] = 0  # Door opening
        
        # Add furniture as obstacles
        # Table
        table_x = start_x + room_width // 2 - 15
        table_y = start_y + room_height // 2 - 10
        map_data[table_y:table_y+20, table_x:table_x+30] = 100
        
        # Chairs
        chair_positions = [
            (table_x - 15, table_y + 5),
            (table_x + 35, table_y + 5),
            (table_x + 10, table_y - 15),
            (table_x + 10, table_y + 25)
        ]
        
        for chair_x, chair_y in chair_positions:
            if 0 <= chair_x < self.map_width - 10 and 0 <= chair_y < self.map_height - 10:
                map_data[chair_y:chair_y+10, chair_x:chair_x+10] = 100
        
        # Add some unknown areas (unexplored regions)
        # Corners and edges
        map_data[0:20, 0:20] = -1  # Top-left corner
        map_data[0:20, -20:] = -1  # Top-right corner
        map_data[-20:, 0:20] = -1  # Bottom-left corner
        map_data[-20:, -20:] = -1  # Bottom-right corner
        
        return map_data

    def create_office_map(self):
        """Create an office building map with multiple rooms"""
        # Initialize as unknown
        map_data = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        
        # Create a central corridor
        corridor_width = 20
        corridor_start_x = (self.map_width - corridor_width) // 2
        map_data[:, corridor_start_x:corridor_start_x+corridor_width] = 0  # Free corridor
        
        # Create rooms on both sides
        room_width = 40
        room_height = 30
        
        # Left side rooms
        for i in range(0, self.map_height - room_height, room_height + 10):
            room_x = corridor_start_x - room_width - 5
            if room_x >= 0:
                # Room interior (free)
                map_data[i+5:i+room_height-5, room_x+5:room_x+room_width-5] = 0
                # Room walls
                map_data[i:i+5, room_x:room_x+room_width] = 100  # Top wall
                map_data[i+room_height-5:i+room_height, room_x:room_x+room_width] = 100  # Bottom wall
                map_data[i:i+room_height, room_x:room_x+5] = 100  # Left wall
                map_data[i:i+room_height, room_x+room_width-5:room_x+room_width] = 100  # Right wall
                
                # Door to corridor
                door_y = i + room_height // 2 - 5
                map_data[door_y:door_y+10, room_x+room_width-5:room_x+room_width] = 0
        
        # Right side rooms
        for i in range(0, self.map_height - room_height, room_height + 10):
            room_x = corridor_start_x + corridor_width + 5
            if room_x + room_width < self.map_width:
                # Room interior (free)
                map_data[i+5:i+room_height-5, room_x+5:room_x+room_width-5] = 0
                # Room walls
                map_data[i:i+5, room_x:room_x+room_width] = 100  # Top wall
                map_data[i+room_height-5:i+room_height, room_x:room_x+room_width] = 100  # Bottom wall
                map_data[i:i+room_height, room_x:room_x+5] = 100  # Left wall
                map_data[i:i+room_height, room_x+room_width-5:room_x+room_width] = 100  # Right wall
                
                # Door to corridor
                door_y = i + room_height // 2 - 5
                map_data[door_y:door_y+10, room_x:room_x+5] = 0
        
        return map_data

    def create_outdoor_map(self):
        """Create an outdoor environment map"""
        # Initialize as free space (outdoor)
        map_data = np.full((self.map_height, self.map_width), 0, dtype=np.int8)
        
        # Add buildings as obstacles
        buildings = [
            (30, 30, 40, 50),   # (x, y, width, height)
            (100, 20, 30, 60),
            (50, 120, 60, 40),
            (130, 100, 35, 45),
            (20, 150, 25, 30)
        ]
        
        for bx, by, bw, bh in buildings:
            if bx + bw < self.map_width and by + bh < self.map_height:
                map_data[by:by+bh, bx:bx+bw] = 100
        
        # Add trees and vegetation as obstacles
        for _ in range(20):
            tree_x = random.randint(10, self.map_width - 10)
            tree_y = random.randint(10, self.map_height - 10)
            tree_size = random.randint(3, 8)
            
            # Make sure tree doesn't overlap with buildings
            if map_data[tree_y, tree_x] == 0:
                map_data[tree_y:tree_y+tree_size, tree_x:tree_x+tree_size] = 100
        
        # Add roads as guaranteed free space
        # Horizontal road
        road_y = self.map_height // 2
        map_data[road_y-10:road_y+10, :] = 0
        
        # Vertical road
        road_x = self.map_width // 2
        map_data[:, road_x-10:road_x+10] = 0
        
        # Add some unknown areas (unexplored regions)
        # Random patches of unknown terrain
        for _ in range(5):
            unknown_x = random.randint(0, self.map_width - 30)
            unknown_y = random.randint(0, self.map_height - 30)
            unknown_w = random.randint(20, 40)
            unknown_h = random.randint(20, 40)
            map_data[unknown_y:unknown_y+unknown_h, unknown_x:unknown_x+unknown_w] = -1
        
        return map_data

    def add_dynamic_elements(self, map_data, current_time):
        """Add some dynamic elements that change over time"""
        # Simulate a moving obstacle (like a person or robot)
        obstacle_x = int(self.map_width/2 + 30 * math.sin(current_time * 0.1))
        obstacle_y = int(self.map_height/2 + 20 * math.cos(current_time * 0.15))
        
        # Ensure obstacle is within bounds
        obstacle_x = max(5, min(self.map_width - 10, obstacle_x))
        obstacle_y = max(5, min(self.map_height - 10, obstacle_y))
        
        # Add the moving obstacle
        map_data[obstacle_y-3:obstacle_y+3, obstacle_x-3:obstacle_x+3] = 100
        
        return map_data

    def publish_occupancy_map(self):
        current_time = time.time() - self.time_offset
        
        # Cycle through different map types
        map_cycle = int(current_time / 60) % 3  # Change every 60 seconds
        
        if map_cycle == 0:
            map_data = self.create_room_map()
            map_type = "Indoor Room"
        elif map_cycle == 1:
            map_data = self.create_office_map()
            map_type = "Office Building"
        else:
            map_data = self.create_outdoor_map()
            map_type = "Outdoor Environment"
        
        # Add dynamic elements
        map_data = self.add_dynamic_elements(map_data, current_time)
        
        # Create OccupancyGrid message
        msg = OccupancyGrid()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Map metadata
        msg.info = MapMetaData()
        msg.info.map_load_time = self.get_clock().now().to_msg()
        msg.info.resolution = self.resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        
        # Origin pose
        msg.info.origin = Pose()
        msg.info.origin.position = Point(x=self.origin_x, y=self.origin_y, z=0.0)
        msg.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Map data (flatten the 2D array)
        msg.data = map_data.flatten().tolist()
        
        # Publish the message
        self.map_pub.publish(msg)
        
        # Calculate statistics
        total_cells = self.map_width * self.map_height
        occupied_cells = np.sum(map_data == 100)
        free_cells = np.sum(map_data == 0)
        unknown_cells = np.sum(map_data == -1)
        
        self.get_logger().info(
            f'Published {map_type} map - '
            f'Occupied: {occupied_cells} ({occupied_cells/total_cells*100:.1f}%), '
            f'Free: {free_cells} ({free_cells/total_cells*100:.1f}%), '
            f'Unknown: {unknown_cells} ({unknown_cells/total_cells*100:.1f}%)'
        )

def main(args=None):
    rclpy.init(args=args)
    
    map_publisher = OccupancyMapTestPublisher()
    
    try:
        rclpy.spin(map_publisher)
    except KeyboardInterrupt:
        map_publisher.get_logger().info('Occupancy Map Test Publisher stopped by user')
    finally:
        map_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
