#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import random
import time
import json

class RobotStatusTestPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_test_publisher')
        
        # Create publishers for robot status topics
        self.status_pub = self.create_publisher(String, '/robot/status', 10)
        self.heartbeat_pub = self.create_publisher(Header, '/robot/heartbeat', 10)
        self.task_pub = self.create_publisher(String, '/robot/current_task', 10)
        self.progress_pub = self.create_publisher(Float32, '/robot/task_progress', 10)
        self.health_pub = self.create_publisher(DiagnosticArray, '/robot/system_health', 10)
        
        # Timer to publish data every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_robot_status)
        
        # Initialize robot state variables
        self.current_status = 'idle'
        self.current_task = 'Standby'
        self.task_progress = 0.0
        self.uptime = 0
        self.status_change_timer = 0
        self.task_scenarios = {
            'idle': ['Standby', 'Waiting for commands', 'System check'],
            'ready': ['Awaiting instructions', 'Ready for mission', 'Systems nominal'],
            'running': ['Navigation to waypoint', 'Object detection', 'Mapping area', 'Charging', 'Patrol route', 'Data collection'],
            'not_responding': ['Connection lost', 'Communication timeout', 'Network error'],
            'error': ['Motor controller fault', 'Sensor calibration failed', 'Navigation path blocked', 'Communication timeout', 'Battery critical', 'Emergency stop activated']
        }
        
        self.get_logger().info('Robot Status Test Publisher started')
        self.get_logger().info('Publishing on topics:')
        self.get_logger().info('  - /robot/status')
        self.get_logger().info('  - /robot/heartbeat')
        self.get_logger().info('  - /robot/current_task')
        self.get_logger().info('  - /robot/task_progress')
        self.get_logger().info('  - /robot/system_health')

    def publish_robot_status(self):
        current_time = time.time()
        self.uptime += 2  # Increment uptime by 2 seconds
        self.status_change_timer += 2
        
        # Change status every 15-30 seconds for demonstration
        if self.status_change_timer >= random.randint(15, 30):
            self.status_change_timer = 0
            self.change_robot_status()
        
        # Update task progress for running status
        if self.current_status == 'running':
            self.task_progress = min(self.task_progress + random.uniform(1, 5), 100.0)
            if self.task_progress >= 100.0:
                self.current_status = 'ready'
                self.current_task = 'Task completed'
                self.task_progress = 0.0
        
        # Publish robot status
        status_msg = String()
        status_msg.data = self.current_status
        self.status_pub.publish(status_msg)
        
        # Publish heartbeat (only if not in not_responding state)
        if self.current_status != 'not_responding':
            heartbeat_msg = Header()
            heartbeat_msg.stamp = self.get_clock().now().to_msg()
            heartbeat_msg.frame_id = 'robot_base'
            self.heartbeat_pub.publish(heartbeat_msg)
        
        # Publish current task
        task_msg = String()
        task_msg.data = self.current_task
        self.task_pub.publish(task_msg)
        
        # Publish task progress
        progress_msg = Float32()
        progress_msg.data = self.task_progress
        self.progress_pub.publish(progress_msg)
        
        # Publish system health
        self.publish_system_health()
        
        # Log status periodically
        if int(current_time) % 10 == 0:  # Log every 10 seconds
            self.get_logger().info(f'Robot Status:')
            self.get_logger().info(f'  Status: {self.current_status}')
            self.get_logger().info(f'  Task: {self.current_task}')
            self.get_logger().info(f'  Progress: {self.task_progress:.1f}%')
            self.get_logger().info(f'  Uptime: {self.uptime}s')

    def change_robot_status(self):
        """Change robot status with realistic transitions"""
        statuses = ['idle', 'ready', 'running', 'not_responding', 'error']
        
        # Define realistic status transitions
        transitions = {
            'idle': ['ready', 'error'],
            'ready': ['running', 'idle', 'error'],
            'running': ['ready', 'error', 'not_responding'],
            'not_responding': ['idle', 'error'],
            'error': ['idle', 'ready']
        }
        
        # Choose next status based on current status
        possible_next = transitions.get(self.current_status, statuses)
        self.current_status = random.choice(possible_next)
        
        # Update task based on new status
        self.current_task = random.choice(self.task_scenarios[self.current_status])
        
        # Reset task progress for new tasks
        if self.current_status == 'running':
            self.task_progress = random.uniform(0, 20)  # Start with some progress
        else:
            self.task_progress = 0.0
        
        self.get_logger().info(f'Status changed to: {self.current_status} - {self.current_task}')

    def publish_system_health(self):
        """Publish system health diagnostics"""
        health_msg = DiagnosticArray()
        health_msg.header.stamp = self.get_clock().now().to_msg()
        health_msg.header.frame_id = 'robot_base'
        
        # Generate realistic health data based on current status
        cpu_usage = self.generate_health_value('cpu')
        memory_usage = self.generate_health_value('memory')
        temperature = self.generate_health_value('temperature')
        disk_usage = self.generate_health_value('disk')
        
        # CPU Status
        cpu_status = DiagnosticStatus()
        cpu_status.name = 'CPU Usage'
        cpu_status.message = str(cpu_usage)
        cpu_status.level = self.get_diagnostic_level(cpu_usage, 'cpu')
        cpu_status.values = [
            KeyValue(key='usage_percent', value=str(cpu_usage)),
            KeyValue(key='cores', value='4'),
            KeyValue(key='frequency', value='2.4GHz')
        ]
        
        # Memory Status
        memory_status = DiagnosticStatus()
        memory_status.name = 'Memory Usage'
        memory_status.message = str(memory_usage)
        memory_status.level = self.get_diagnostic_level(memory_usage, 'memory')
        memory_status.values = [
            KeyValue(key='usage_percent', value=str(memory_usage)),
            KeyValue(key='total_gb', value='8'),
            KeyValue(key='available_gb', value=str(8 * (100 - memory_usage) / 100))
        ]
        
        # Temperature Status
        temp_status = DiagnosticStatus()
        temp_status.name = 'System Temperature'
        temp_status.message = str(temperature)
        temp_status.level = self.get_diagnostic_level(temperature, 'temperature')
        temp_status.values = [
            KeyValue(key='temperature_celsius', value=str(temperature)),
            KeyValue(key='max_safe_temp', value='85'),
            KeyValue(key='sensor_location', value='CPU')
        ]
        
        # Disk Status
        disk_status = DiagnosticStatus()
        disk_status.name = 'Disk Usage'
        disk_status.message = str(disk_usage)
        disk_status.level = self.get_diagnostic_level(disk_usage, 'disk')
        disk_status.values = [
            KeyValue(key='usage_percent', value=str(disk_usage)),
            KeyValue(key='total_gb', value='256'),
            KeyValue(key='available_gb', value=str(256 * (100 - disk_usage) / 100))
        ]
        
        health_msg.status = [cpu_status, memory_status, temp_status, disk_status]
        self.health_pub.publish(health_msg)

    def generate_health_value(self, metric_type):
        """Generate realistic health values based on robot status"""
        base_values = {
            'cpu': 20,
            'memory': 30,
            'temperature': 45,
            'disk': 25
        }
        
        base = base_values[metric_type]
        
        # Adjust values based on current status
        if self.current_status == 'running':
            base += random.uniform(20, 40)  # Higher usage when running
        elif self.current_status == 'error':
            base += random.uniform(30, 50)  # Very high usage during errors
        elif self.current_status == 'not_responding':
            base += random.uniform(40, 60)  # Critical levels when not responding
        else:
            base += random.uniform(-5, 15)  # Normal variation
        
        # Add some noise
        value = base + random.uniform(-5, 5)
        
        # Clamp values to realistic ranges
        if metric_type == 'temperature':
            return max(25, min(95, value))  # 25-95°C
        else:
            return max(0, min(100, value))  # 0-100%

    def get_diagnostic_level(self, value, metric_type):
        """Get diagnostic level based on value and metric type"""
        if metric_type == 'temperature':
            if value < 60:
                return DiagnosticStatus.OK
            elif value < 80:
                return DiagnosticStatus.WARN
            else:
                return DiagnosticStatus.ERROR
        else:  # cpu, memory, disk
            if value < 50:
                return DiagnosticStatus.OK
            elif value < 80:
                return DiagnosticStatus.WARN
            else:
                return DiagnosticStatus.ERROR

def main(args=None):
    rclpy.init(args=args)
    
    try:
        publisher = RobotStatusTestPublisher()
        
        print("🤖 Robot Status Test Publisher")
        print("==============================")
        print("Publishing realistic robot status data...")
        print("Status States:")
        print("  🟦 IDLE - Robot is idle and waiting for commands")
        print("  🟢 READY - Robot is ready to execute tasks")
        print("  🟡 RUNNING - Robot is actively executing a task")
        print("  🟠 NOT RESPONDING - Robot is not responding to commands")
        print("  🔴 ERROR - Robot has encountered an error")
        print("\nSystem Health Monitoring:")
        print("  🖥️  CPU Usage")
        print("  💾 Memory Usage")
        print("  🌡️  System Temperature")
        print("  💿 Disk Usage")
        print("\nPress Ctrl+C to stop...")
        
        rclpy.spin(publisher)
        
    except KeyboardInterrupt:
        print("\n🛑 Shutting down Robot Status Test Publisher...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'publisher' in locals():
            publisher.destroy_node()
        rclpy.shutdown()
        print("✅ Robot Status Test Publisher stopped")

if __name__ == '__main__':
    main()
