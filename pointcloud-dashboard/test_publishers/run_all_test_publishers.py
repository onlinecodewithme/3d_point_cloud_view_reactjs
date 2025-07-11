#!/usr/bin/env python3

"""
Master Test Publisher Launcher for ROS2 Dashboard Testing
This script launches all test publishers simultaneously to provide comprehensive test data
for the React dashboard application.

Usage:
    python3 run_all_test_publishers.py [options]

Options:
    --publishers LIST    Comma-separated list of publishers to run (default: all)
    --no-battery        Skip battery test publisher
    --no-imu           Skip IMU test publisher
    --no-system        Skip system monitoring test publisher
    --no-pointcloud    Skip point cloud test publisher
    --no-map           Skip occupancy map test publisher
    --no-camera        Skip camera test publisher
    --no-robot         Skip robot control test publisher
    --list             List available publishers and exit

Make sure you have ROS2 and the rosbridge_suite running:
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
"""

import subprocess
import sys
import time
import signal
import argparse
import os
from pathlib import Path

class TestPublisherLauncher:
    def __init__(self):
        self.processes = []
        self.script_dir = Path(__file__).parent
        
        # Available publishers
        self.publishers = {
            'battery': {
                'script': 'battery_test_publisher.py',
                'description': 'Battery monitoring data (voltage, current, SOC, temperature)',
                'topics': ['/battery/soc', '/battery/pack_voltage', '/battery/current', '/battery/temperature']
            },
            'imu': {
                'script': 'imu_test_publisher.py', 
                'description': 'IMU sensor data (orientation, acceleration, gyroscope, magnetometer)',
                'topics': ['/perception_brain/imu/json']
            },
            'system': {
                'script': 'system_monitoring_test_publisher.py',
                'description': 'System monitoring for cognition and perception brains',
                'topics': ['/cognitionbrain/system_monitoring', '/perceptionbrain/system_monitoring']
            },
            'pointcloud': {
                'script': 'pointcloud_test_publisher.py',
                'description': '3D point cloud data with realistic indoor/outdoor scenes',
                'topics': ['/cloud_map', '/rtabmap/cloud_map', '/zed/zed_node/point_cloud/cloud_registered']
            },
            'map': {
                'script': 'occupancy_map_test_publisher.py',
                'description': 'Occupancy grid maps for navigation (room, office, outdoor)',
                'topics': ['/map']
            },
            'camera': {
                'script': 'camera_test_publisher.py',
                'description': 'Camera feeds with synthetic scenes (stereo, depth, compressed)',
                'topics': ['/zed/zed_node/left/image_rect_color', '/zed/zed_node/right/image_rect_color', '/camera/image_raw']
            },
            'robot': {
                'script': 'robot_control_test_publisher.py',
                'description': 'Robot control, odometry, and navigation status',
                'topics': ['/odom', '/robot_status', '/joy']
            }
        }

    def signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        print(f"\n🛑 Received signal {signum}. Shutting down all test publishers...")
        self.stop_all_publishers()
        sys.exit(0)

    def start_publisher(self, name, script_name):
        """Start a single publisher"""
        script_path = self.script_dir / script_name
        
        if not script_path.exists():
            print(f"❌ Error: Script {script_name} not found at {script_path}")
            return None
        
        try:
            print(f"🚀 Starting {name} publisher...")
            process = subprocess.Popen([
                sys.executable, str(script_path)
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=os.environ.copy())
            
            # Give it a moment to start
            time.sleep(0.5)
            
            # Check if process is still running
            if process.poll() is None:
                print(f"✅ {name} publisher started successfully (PID: {process.pid})")
                return process
            else:
                stdout, stderr = process.communicate()
                print(f"❌ {name} publisher failed to start:")
                if stderr:
                    print(f"   Error: {stderr}")
                return None
                
        except Exception as e:
            print(f"❌ Failed to start {name} publisher: {e}")
            return None

    def stop_all_publishers(self):
        """Stop all running publishers"""
        for process in self.processes:
            if process and process.poll() is None:
                try:
                    process.terminate()
                    # Give it 2 seconds to terminate gracefully
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill()
                except Exception as e:
                    print(f"⚠️  Error stopping process: {e}")
        
        self.processes.clear()
        print("🔴 All test publishers stopped")

    def list_publishers(self):
        """List all available publishers"""
        print("📋 Available Test Publishers:")
        print("=" * 60)
        
        for name, info in self.publishers.items():
            print(f"\n🔧 {name.upper()}")
            print(f"   Description: {info['description']}")
            print(f"   Script: {info['script']}")
            print(f"   Topics: {', '.join(info['topics'])}")
        
        print(f"\n📊 Total: {len(self.publishers)} publishers available")

    def check_dependencies(self):
        """Check if required dependencies are available"""
        print("🔍 Checking dependencies...")
        
        # Check if ROS2 environment is sourced
        ros_distro = os.environ.get('ROS_DISTRO')
        if ros_distro:
            print(f"✅ ROS2 environment found: {ros_distro}")
        else:
            # Fallback: try to run ros2 command
            try:
                result = subprocess.run(['ros2', '--version'], capture_output=True, text=True, env=os.environ.copy())
                if result.returncode == 0:
                    print(f"✅ ROS2 found: {result.stdout.strip()}")
                else:
                    print("❌ ROS2 not found. Please install ROS2 and source the setup.")
                    return False
            except FileNotFoundError:
                print("❌ ROS2 not found. Please install ROS2 and source the setup.")
                return False
        
        # Check if rosbridge is running
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=5, env=os.environ.copy())
            if result.returncode == 0:
                print("✅ ROS2 system is running")
            else:
                print("⚠️  ROS2 system may not be fully initialized")
        except (subprocess.TimeoutExpired, FileNotFoundError):
            print("⚠️  Could not verify ROS2 system status")
        
        # Check Python dependencies
        required_packages = ['rclpy', 'numpy', 'cv2']
        missing_packages = []
        
        for package in required_packages:
            try:
                __import__(package)
                print(f"✅ {package} found")
            except ImportError:
                missing_packages.append(package)
                print(f"❌ {package} not found")
        
        if missing_packages:
            print(f"\n⚠️  Missing packages: {', '.join(missing_packages)}")
            print("Install with: pip install rclpy numpy opencv-python")
            return False
        
        return True

    def run(self, selected_publishers=None):
        """Run the selected publishers"""
        # Set up signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Check dependencies
        if not self.check_dependencies():
            print("\n❌ Dependency check failed. Please resolve the issues above.")
            return False
        
        # Determine which publishers to run
        if selected_publishers is None:
            selected_publishers = list(self.publishers.keys())
        
        print(f"\n🎯 Starting {len(selected_publishers)} test publishers...")
        print("=" * 60)
        
        # Start publishers
        for name in selected_publishers:
            if name not in self.publishers:
                print(f"⚠️  Unknown publisher: {name}")
                continue
            
            process = self.start_publisher(name, self.publishers[name]['script'])
            if process:
                self.processes.append(process)
        
        if not self.processes:
            print("❌ No publishers started successfully")
            return False
        
        print(f"\n🎉 Successfully started {len(self.processes)} publishers!")
        print("\n📡 Published Topics:")
        for name in selected_publishers:
            if name in self.publishers:
                for topic in self.publishers[name]['topics']:
                    print(f"   • {topic}")
        
        print("\n🌐 Dashboard URLs:")
        print("   • Main Dashboard: http://localhost:3000")
        print("   • Battery Monitor: http://localhost:3000/battery")
        print("   • System Monitor: http://localhost:3000/system-monitoring")
        print("   • IMU Sensor: http://localhost:3000/imu-sensor")
        print("   • Robot Control: http://localhost:3000/control")
        
        print("\n📋 Monitoring Status:")
        print("   Press Ctrl+C to stop all publishers")
        print("   Check ROS topics with: ros2 topic list")
        print("   Monitor data with: ros2 topic echo <topic_name>")
        
        # Monitor processes
        try:
            while True:
                time.sleep(5)
                
                # Check if any process has died
                active_processes = []
                for i, process in enumerate(self.processes):
                    if process.poll() is None:
                        active_processes.append(process)
                    else:
                        print(f"⚠️  Publisher process {i} has stopped")
                
                self.processes = active_processes
                
                if not self.processes:
                    print("❌ All publishers have stopped")
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_all_publishers()
        
        return True

def main():
    parser = argparse.ArgumentParser(
        description='Launch test publishers for ROS2 dashboard testing',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 run_all_test_publishers.py                    # Run all publishers
  python3 run_all_test_publishers.py --list             # List available publishers
  python3 run_all_test_publishers.py --publishers battery,imu,system  # Run specific publishers
  python3 run_all_test_publishers.py --no-camera --no-pointcloud      # Run all except camera and pointcloud
        """
    )
    
    parser.add_argument('--publishers', type=str, 
                       help='Comma-separated list of publishers to run')
    parser.add_argument('--no-battery', action='store_true',
                       help='Skip battery test publisher')
    parser.add_argument('--no-imu', action='store_true',
                       help='Skip IMU test publisher')
    parser.add_argument('--no-system', action='store_true',
                       help='Skip system monitoring test publisher')
    parser.add_argument('--no-pointcloud', action='store_true',
                       help='Skip point cloud test publisher')
    parser.add_argument('--no-map', action='store_true',
                       help='Skip occupancy map test publisher')
    parser.add_argument('--no-camera', action='store_true',
                       help='Skip camera test publisher')
    parser.add_argument('--no-robot', action='store_true',
                       help='Skip robot control test publisher')
    parser.add_argument('--list', action='store_true',
                       help='List available publishers and exit')
    
    args = parser.parse_args()
    
    launcher = TestPublisherLauncher()
    
    if args.list:
        launcher.list_publishers()
        return
    
    # Determine which publishers to run
    selected_publishers = None
    
    if args.publishers:
        selected_publishers = [p.strip() for p in args.publishers.split(',')]
    else:
        # Start with all publishers
        selected_publishers = list(launcher.publishers.keys())
        
        # Remove excluded publishers
        if args.no_battery and 'battery' in selected_publishers:
            selected_publishers.remove('battery')
        if args.no_imu and 'imu' in selected_publishers:
            selected_publishers.remove('imu')
        if args.no_system and 'system' in selected_publishers:
            selected_publishers.remove('system')
        if args.no_pointcloud and 'pointcloud' in selected_publishers:
            selected_publishers.remove('pointcloud')
        if args.no_map and 'map' in selected_publishers:
            selected_publishers.remove('map')
        if args.no_camera and 'camera' in selected_publishers:
            selected_publishers.remove('camera')
        if args.no_robot and 'robot' in selected_publishers:
            selected_publishers.remove('robot')
    
    if not selected_publishers:
        print("❌ No publishers selected to run")
        return
    
    # Validate selected publishers
    invalid_publishers = [p for p in selected_publishers if p not in launcher.publishers]
    if invalid_publishers:
        print(f"❌ Invalid publishers: {', '.join(invalid_publishers)}")
        print("Use --list to see available publishers")
        return
    
    print("🚀 ROS2 Dashboard Test Publisher Launcher")
    print("=" * 50)
    
    success = launcher.run(selected_publishers)
    
    if success:
        print("\n✅ Test publishers completed successfully")
    else:
        print("\n❌ Test publishers failed")
        sys.exit(1)

if __name__ == '__main__':
    main()
