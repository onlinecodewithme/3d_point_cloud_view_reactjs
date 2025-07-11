#!/usr/bin/env python3

"""
System Monitoring Test Data Publisher for ROS2
This script publishes mock system monitoring data for both cognition and perception brains
to test the enhanced system monitoring dashboard.

Usage:
    python3 system_monitoring_test_publisher.py

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
import psutil

class SystemMonitoringTestPublisher(Node):
    def __init__(self):
        super().__init__('system_monitoring_test_publisher')
        
        # Create publishers for both brain systems
        self.cognition_pub = self.create_publisher(String, '/cognitionbrain/system_monitoring', 10)
        self.perception_pub = self.create_publisher(String, '/perceptionbrain/system_monitoring', 10)
        
        # Timer to publish data every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_system_data)
        
        # Initialize system state
        self.time_offset = time.time()
        self.cognition_base_cpu = 45.0
        self.perception_base_cpu = 65.0
        self.cognition_base_memory = 60.0
        self.perception_base_memory = 75.0
        
        self.get_logger().info('System Monitoring Test Publisher started')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  /cognitionbrain/system_monitoring')
        self.get_logger().info('  /perceptionbrain/system_monitoring')

    def generate_system_data(self, brain_type, base_cpu, base_memory):
        """Generate realistic system monitoring data"""
        current_time = time.time() - self.time_offset
        
        # Generate realistic variations
        cpu_variation = math.sin(current_time * 0.1) * 15 + random.uniform(-5, 5)
        memory_variation = math.sin(current_time * 0.05) * 10 + random.uniform(-3, 3)
        
        cpu_usage = max(5, min(95, base_cpu + cpu_variation))
        memory_usage = max(20, min(90, base_memory + memory_variation))
        
        # Temperature varies with CPU usage
        temperature = 35 + (cpu_usage / 100) * 30 + random.uniform(-2, 2)
        
        # Disk usage slowly increases
        disk_usage = 45 + (current_time / 3600) * 0.1 + random.uniform(-1, 1)
        disk_usage = max(30, min(85, disk_usage))
        
        # Network activity
        network_rx = random.uniform(1000, 50000)  # bytes/s
        network_tx = random.uniform(500, 25000)   # bytes/s
        
        # Process count varies
        process_count = random.randint(150, 300)
        
        # Uptime increases
        uptime = current_time + random.uniform(3600, 86400)  # 1 hour to 1 day base
        
        # Load averages
        load_1min = cpu_usage / 100 * 4 + random.uniform(-0.5, 0.5)
        load_5min = cpu_usage / 100 * 3.5 + random.uniform(-0.3, 0.3)
        load_15min = cpu_usage / 100 * 3 + random.uniform(-0.2, 0.2)
        
        system_data = {
            "timestamp": current_time,
            "brain_type": brain_type,
            "cpu": {
                "usage": round(cpu_usage, 2),
                "usage_percent": round(cpu_usage, 2),
                "temperature": round(temperature, 1),
                "cores": 8,
                "frequency": round(2400 + random.uniform(-200, 400), 0)
            },
            "memory": {
                "usage": round(memory_usage, 2),
                "usage_percent": round(memory_usage, 2),
                "total_gb": 16.0,
                "available_gb": round(16.0 * (100 - memory_usage) / 100, 2),
                "used_gb": round(16.0 * memory_usage / 100, 2)
            },
            "disk": {
                "usage": round(disk_usage, 2),
                "usage_percent": round(disk_usage, 2),
                "total_gb": 512.0,
                "available_gb": round(512.0 * (100 - disk_usage) / 100, 1),
                "used_gb": round(512.0 * disk_usage / 100, 1)
            },
            "storage": {
                "usage_percent": round(disk_usage, 2)
            },
            "thermal": {
                "temperature": round(temperature, 1)
            },
            "network": {
                "rx_bytes": int(network_rx),
                "tx_bytes": int(network_tx),
                "bytes_received": int(network_rx),
                "bytes_sent": int(network_tx)
            },
            "system": {
                "process_count": process_count,
                "uptime": int(uptime),
                "load_average": [
                    round(max(0, load_1min), 2),
                    round(max(0, load_5min), 2),
                    round(max(0, load_15min), 2)
                ]
            },
            "system_load": {
                "load_1min": round(max(0, load_1min), 2),
                "load_5min": round(max(0, load_5min), 2),
                "load_15min": round(max(0, load_15min), 2)
            },
            "processes": {
                "process_count": process_count,
                "running": random.randint(5, 15),
                "sleeping": process_count - random.randint(5, 15),
                "zombie": random.randint(0, 2)
            },
            "gpu": {
                "usage": round(random.uniform(20, 80), 1),
                "memory_usage": round(random.uniform(30, 70), 1),
                "temperature": round(temperature + random.uniform(5, 15), 1)
            } if brain_type == "perception" else None
        }
        
        # Remove None values
        if system_data["gpu"] is None:
            del system_data["gpu"]
            
        return system_data

    def publish_system_data(self):
        # Generate and publish cognition brain data
        cognition_data = self.generate_system_data("cognition", self.cognition_base_cpu, self.cognition_base_memory)
        cognition_msg = String()
        cognition_msg.data = json.dumps(cognition_data)
        self.cognition_pub.publish(cognition_msg)
        
        # Generate and publish perception brain data
        perception_data = self.generate_system_data("perception", self.perception_base_cpu, self.perception_base_memory)
        perception_msg = String()
        perception_msg.data = json.dumps(perception_data)
        self.perception_pub.publish(perception_msg)
        
        # Log current values
        self.get_logger().info(
            f'Published System Data - '
            f'Cognition: CPU={cognition_data["cpu"]["usage"]:.1f}%, '
            f'Memory={cognition_data["memory"]["usage"]:.1f}%, '
            f'Temp={cognition_data["cpu"]["temperature"]:.1f}°C | '
            f'Perception: CPU={perception_data["cpu"]["usage"]:.1f}%, '
            f'Memory={perception_data["memory"]["usage"]:.1f}%, '
            f'Temp={perception_data["cpu"]["temperature"]:.1f}°C'
        )
        
        # Occasionally vary the base values to simulate different workloads
        if random.random() < 0.1:  # 10% chance every 2 seconds
            self.cognition_base_cpu += random.uniform(-5, 5)
            self.cognition_base_cpu = max(20, min(80, self.cognition_base_cpu))
            
            self.perception_base_cpu += random.uniform(-5, 5)
            self.perception_base_cpu = max(30, min(90, self.perception_base_cpu))
            
            self.cognition_base_memory += random.uniform(-3, 3)
            self.cognition_base_memory = max(40, min(85, self.cognition_base_memory))
            
            self.perception_base_memory += random.uniform(-3, 3)
            self.perception_base_memory = max(50, min(90, self.perception_base_memory))

def main(args=None):
    rclpy.init(args=args)
    
    system_publisher = SystemMonitoringTestPublisher()
    
    try:
        rclpy.spin(system_publisher)
    except KeyboardInterrupt:
        system_publisher.get_logger().info('System Monitoring Test Publisher stopped by user')
    finally:
        system_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
