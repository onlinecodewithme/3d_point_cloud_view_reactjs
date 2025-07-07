#!/usr/bin/env python3

"""
Battery Test Data Publisher for ROS2
This script publishes mock battery data to test the battery dashboard.

Usage:
    python3 battery_test_publisher.py

Make sure you have ROS2 and the rosbridge_suite running:
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Float64MultiArray
import random
import math
import time

class BatteryTestPublisher(Node):
    def __init__(self):
        super().__init__('battery_test_publisher')
        
        # Create publishers for each battery topic
        self.soc_pub = self.create_publisher(Float64, '/battery/soc', 10)
        self.voltage_pub = self.create_publisher(Float64, '/battery/pack_voltage', 10)
        self.current_pub = self.create_publisher(Float64, '/battery/current', 10)
        self.remaining_capacity_pub = self.create_publisher(Float64, '/battery/remaining_capacity', 10)
        self.total_capacity_pub = self.create_publisher(Float64, '/battery/total_capacity', 10)
        self.cycles_pub = self.create_publisher(Float64, '/battery/cycles', 10)
        self.temperature_pub = self.create_publisher(Float64MultiArray, '/battery/temperature', 10)
        self.status_pub = self.create_publisher(String, '/battery/status', 10)
        self.state_pub = self.create_publisher(String, '/battery/state', 10)
        self.cell_voltages_pub = self.create_publisher(Float64MultiArray, '/battery/cell_voltages', 10)
        
        # Timer to publish data every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_battery_data)
        
        # Initialize battery state
        self.base_soc = 85.0  # Starting at 85%
        self.base_voltage = 53.0  # Starting voltage
        self.cycle_count = 1
        self.total_capacity = 230.0  # Ah
        
        self.get_logger().info('Battery Test Publisher started. Publishing to battery topics...')
        self.get_logger().info('Topics being published:')
        self.get_logger().info('  /battery/soc')
        self.get_logger().info('  /battery/pack_voltage')
        self.get_logger().info('  /battery/current')
        self.get_logger().info('  /battery/remaining_capacity')
        self.get_logger().info('  /battery/total_capacity')
        self.get_logger().info('  /battery/cycles')
        self.get_logger().info('  /battery/temperature')
        self.get_logger().info('  /battery/status')
        self.get_logger().info('  /battery/state')
        self.get_logger().info('  /battery/cell_voltages')

    def publish_battery_data(self):
        # Generate realistic battery data with some variation
        current_time = time.time()
        
        # SOC varies slowly between 75-95%
        soc_variation = math.sin(current_time * 0.1) * 10 + 85
        soc = max(75, min(95, soc_variation + random.uniform(-2, 2)))
        
        # Pack voltage correlates with SOC
        pack_voltage = 50.0 + (soc / 100.0) * 6.0 + random.uniform(-0.5, 0.5)
        
        # Current varies between -3A (discharging) and +2A (charging)
        current = math.sin(current_time * 0.05) * 2.5 + random.uniform(-1, 1)
        
        # Remaining capacity based on SOC
        remaining_capacity = (soc / 100.0) * self.total_capacity
        
        # Temperature varies around 30-35°C
        temp1 = 32 + math.sin(current_time * 0.02) * 3 + random.uniform(-1, 1)
        temp2 = 30 + math.sin(current_time * 0.03) * 4 + random.uniform(-1, 1)
        
        # Cell voltages (16 cells, around 3.3V each)
        base_cell_voltage = pack_voltage / 16.0
        cell_voltages = []
        for i in range(16):
            cell_voltage = base_cell_voltage + random.uniform(-0.05, 0.05)
            cell_voltages.append(cell_voltage)
        
        # Battery status based on current
        if current > 0.5:
            status = "Charging"
        elif current < -0.5:
            status = "Discharging"
        else:
            status = "Idle"
        
        # Publish all data
        soc_msg = Float64()
        soc_msg.data = soc
        self.soc_pub.publish(soc_msg)
        
        voltage_msg = Float64()
        voltage_msg.data = pack_voltage
        self.voltage_pub.publish(voltage_msg)
        
        current_msg = Float64()
        current_msg.data = current
        self.current_pub.publish(current_msg)
        
        remaining_msg = Float64()
        remaining_msg.data = remaining_capacity
        self.remaining_capacity_pub.publish(remaining_msg)
        
        total_msg = Float64()
        total_msg.data = self.total_capacity
        self.total_capacity_pub.publish(total_msg)
        
        cycles_msg = Float64()
        cycles_msg.data = float(self.cycle_count)
        self.cycles_pub.publish(cycles_msg)
        
        temp_msg = Float64MultiArray()
        temp_msg.data = [temp1, temp2]
        self.temperature_pub.publish(temp_msg)
        
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
        
        state_msg = String()
        state_msg.data = "Normal"
        self.state_pub.publish(state_msg)
        
        cell_msg = Float64MultiArray()
        cell_msg.data = cell_voltages
        self.cell_voltages_pub.publish(cell_msg)
        
        # Log current values
        self.get_logger().info(f'Published: SOC={soc:.1f}%, Voltage={pack_voltage:.2f}V, Current={current:.2f}A, Status={status}')

def main(args=None):
    rclpy.init(args=args)
    
    battery_publisher = BatteryTestPublisher()
    
    try:
        rclpy.spin(battery_publisher)
    except KeyboardInterrupt:
        battery_publisher.get_logger().info('Battery Test Publisher stopped by user')
    finally:
        battery_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
