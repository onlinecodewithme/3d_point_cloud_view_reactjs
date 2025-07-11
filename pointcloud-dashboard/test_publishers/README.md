# ROS2 Dashboard Test Publishers

This directory contains comprehensive test publishers for all integrations in the React dashboard application. These publishers generate realistic mock data to test and demonstrate all dashboard features without requiring actual hardware.

## 📋 Overview

The test publishers simulate a complete robotic system with:
- **Battery Management System** - Power monitoring and health
- **IMU Sensor** - Orientation, acceleration, and magnetic field data
- **System Monitoring** - CPU, memory, temperature for dual-brain architecture
- **3D Point Cloud** - LIDAR/camera-based 3D mapping data
- **Occupancy Maps** - 2D navigation maps with obstacles
- **Camera Feeds** - Stereo vision and depth sensing
- **Robot Control** - Odometry, navigation, and joystick input

## 🚀 Quick Start

### Prerequisites

1. **ROS2 Installation** (Humble/Iron/Jazzy)
   ```bash
   # Ubuntu/Debian
   sudo apt update
   sudo apt install ros-humble-desktop
   source /opt/ros/humble/setup.bash
   
   # Or use your ROS2 distribution
   ```

2. **Python Dependencies**
   ```bash
   pip install rclpy numpy opencv-python
   ```

3. **ROS Bridge Suite**
   ```bash
   sudo apt install ros-humble-rosbridge-suite
   ```

### Running All Test Publishers

**🚀 Easy Method (Recommended):**
```bash
cd pointcloud-dashboard/test_publishers
./start_test_publishers.sh
```

**📋 Manual Method:**

1. **Start ROS Bridge** (in terminal 1):
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

2. **Start Test Publishers** (in terminal 2):
   ```bash
   cd pointcloud-dashboard/test_publishers
   source /opt/ros/humble/setup.bash
   python3 run_all_test_publishers.py
   ```

3. **Start React Dashboard** (in terminal 3):
   ```bash
   cd pointcloud-dashboard
   npm start
   ```

4. **Open Dashboard**: http://localhost:3000

## 📊 Individual Test Publishers

### 1. Battery Test Publisher
**File**: `battery_test_publisher.py`
**Topics**: 
- `/battery/soc` - State of charge (%)
- `/battery/pack_voltage` - Pack voltage (V)
- `/battery/current` - Current flow (A)
- `/battery/remaining_capacity` - Remaining capacity (Ah)
- `/battery/total_capacity` - Total capacity (Ah)
- `/battery/cycles` - Charge cycles
- `/battery/temperature` - Cell temperatures (°C)
- `/battery/status` - Charging status
- `/battery/state` - Battery health state
- `/battery/cell_voltages` - Individual cell voltages

**Features**:
- Realistic battery discharge/charge cycles
- Temperature correlation with usage
- Cell voltage variations
- Multiple battery states (charging, discharging, idle)

**Usage**:
```bash
python3 battery_test_publisher.py
```

### 2. IMU Sensor Test Publisher
**File**: `imu_test_publisher.py`
**Topics**:
- `/perception_brain/imu/json` - Complete IMU data in JSON format

**Data Includes**:
- Quaternion orientation (w, x, y, z)
- Angular velocity (gyroscope) in rad/s
- Linear acceleration in m/s²
- Magnetometer readings in μT
- Gravity vector
- Temperature and calibration status

**Features**:
- Realistic motion patterns (roll, pitch, yaw)
- Sensor noise simulation
- Magnetic declination handling
- High-frequency updates (20 Hz)

**Usage**:
```bash
python3 imu_test_publisher.py
```

### 3. System Monitoring Test Publisher
**File**: `system_monitoring_test_publisher.py`
**Topics**:
- `/cognitionbrain/system_monitoring` - Cognition brain metrics
- `/perceptionbrain/system_monitoring` - Perception brain metrics

**Metrics Monitored**:
- CPU usage and temperature
- Memory usage (RAM)
- Disk/storage usage
- Network I/O (bytes sent/received)
- Process count and load averages
- System uptime
- GPU metrics (for perception brain)

**Features**:
- Dual-brain architecture simulation
- Realistic resource usage patterns
- Correlated metrics (CPU vs temperature)
- Dynamic workload simulation

**Usage**:
```bash
python3 system_monitoring_test_publisher.py
```

### 4. Point Cloud Test Publisher
**File**: `pointcloud_test_publisher.py`
**Topics**:
- `/cloud_map` - Main point cloud topic
- `/rtabmap/cloud_map` - RTAB-Map point cloud
- `/zed/zed_node/point_cloud/cloud_registered` - ZED camera point cloud

**Scene Types**:
- **Indoor Room**: Furniture, walls, moving person
- **Outdoor Terrain**: Hills, trees, buildings, moving vehicle

**Features**:
- RGB colored point clouds
- Dynamic objects (people, vehicles)
- Realistic scene geometry
- Alternating environments (30-second cycles)
- 5000+ points per cloud at 10 Hz

**Usage**:
```bash
python3 pointcloud_test_publisher.py
```

### 5. Occupancy Map Test Publisher
**File**: `occupancy_map_test_publisher.py`
**Topics**:
- `/map` - Navigation occupancy grid

**Map Types**:
- **Indoor Room**: Single room with furniture and doors
- **Office Building**: Multi-room layout with corridors
- **Outdoor Environment**: Buildings, roads, and vegetation

**Features**:
- Standard occupancy grid format (0=free, 100=occupied, -1=unknown)
- Dynamic obstacles (moving objects)
- Realistic map resolution (0.1 m/cell)
- Map cycling (60-second intervals)

**Usage**:
```bash
python3 occupancy_map_test_publisher.py
```

### 6. Camera Test Publisher
**File**: `camera_test_publisher.py`
**Topics**:
- `/zed/zed_node/left/image_rect_color` - Left stereo camera
- `/zed/zed_node/right/image_rect_color` - Right stereo camera
- `/zed/zed_node/depth/depth_registered` - Depth camera
- `/camera/image_raw` - Raw camera feed
- `/camera/image_raw/compressed` - Compressed JPEG feed

**Scene Types**:
- **Outdoor**: Buildings, moving cars, pedestrians, particles
- **Indoor**: Room with furniture, moving robot

**Features**:
- Synthetic scene generation with OpenCV
- Stereo camera simulation (parallax offset)
- Depth visualization
- Moving objects and dynamic elements
- 640x480 resolution at 30 FPS
- Compressed and uncompressed formats

**Usage**:
```bash
python3 camera_test_publisher.py
```

### 7. Robot Control Test Publisher
**File**: `robot_control_test_publisher.py`
**Topics Published**:
- `/odom` - Robot odometry (position, velocity)
- `/robot_status` - Comprehensive robot status (JSON)
- `/joy` - Simulated joystick input

**Topics Subscribed**:
- `/cmd_vel` - Velocity commands (Twist)
- `/move_base_simple/goal` - Navigation goals
- `/initialpose` - Initial pose setting

**Features**:
- Realistic robot kinematics simulation
- Navigation goal tracking
- Joystick input simulation
- Comprehensive status reporting
- Interactive command response

**Usage**:
```bash
python3 robot_control_test_publisher.py
```

## 🎛️ Master Launcher

The `run_all_test_publishers.py` script provides centralized control over all test publishers.

### Command Line Options

```bash
# Run all publishers
python3 run_all_test_publishers.py

# List available publishers
python3 run_all_test_publishers.py --list

# Run specific publishers
python3 run_all_test_publishers.py --publishers battery,imu,system

# Exclude specific publishers
python3 run_all_test_publishers.py --no-camera --no-pointcloud

# Individual exclusions
python3 run_all_test_publishers.py --no-battery    # Skip battery
python3 run_all_test_publishers.py --no-imu       # Skip IMU
python3 run_all_test_publishers.py --no-system    # Skip system monitoring
python3 run_all_test_publishers.py --no-pointcloud # Skip point cloud
python3 run_all_test_publishers.py --no-map       # Skip occupancy map
python3 run_all_test_publishers.py --no-camera    # Skip camera feeds
python3 run_all_test_publishers.py --no-robot     # Skip robot control
```

### Features

- **Dependency Checking**: Verifies ROS2 and Python packages
- **Process Management**: Graceful startup and shutdown
- **Status Monitoring**: Tracks publisher health
- **Signal Handling**: Clean shutdown with Ctrl+C
- **Comprehensive Logging**: Detailed status information

## 🔧 Testing Individual Components

### Battery Dashboard
```bash
# Start battery publisher
python3 battery_test_publisher.py

# Open dashboard
http://localhost:3000/battery
```

### IMU Sensor Dashboard
```bash
# Start IMU publisher
python3 imu_test_publisher.py

# Open dashboard
http://localhost:3000/imu-sensor
```

### System Monitoring Dashboard
```bash
# Start system monitoring publisher
python3 system_monitoring_test_publisher.py

# Open dashboard
http://localhost:3000/system-monitoring
```

### 3D Point Cloud Visualization
```bash
# Start point cloud publisher
python3 pointcloud_test_publisher.py

# Open main dashboard
http://localhost:3000
```

### Robot Control Dashboard
```bash
# Start robot control publisher
python3 robot_control_test_publisher.py

# Open control dashboard
http://localhost:3000/control
```

## 📡 ROS Topic Monitoring

Monitor published topics in real-time:

```bash
# List all active topics
ros2 topic list

# Monitor specific topic
ros2 topic echo /battery/soc
ros2 topic echo /perception_brain/imu/json
ros2 topic echo /cognitionbrain/system_monitoring

# Check topic frequency
ros2 topic hz /cloud_map
ros2 topic hz /zed/zed_node/left/image_rect_color

# Topic information
ros2 topic info /map
ros2 topic info /odom
```

## 🐛 Troubleshooting

### Common Issues

1. **ROS Bridge Not Running**
   ```
   Error: Connection error: [Errno 111] Connection refused
   ```
   **Solution**: Start rosbridge_server first
   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

2. **Missing Python Dependencies**
   ```
   ImportError: No module named 'rclpy'
   ```
   **Solution**: Install required packages
   ```bash
   pip install rclpy numpy opencv-python
   ```

3. **Permission Denied**
   ```
   Permission denied: './battery_test_publisher.py'
   ```
   **Solution**: Make scripts executable
   ```bash
   chmod +x *.py
   ```

4. **Port Already in Use**
   ```
   Error: Address already in use
   ```
   **Solution**: Kill existing processes
   ```bash
   pkill -f rosbridge
   pkill -f test_publisher
   ```

5. **ROS2 Environment Not Found** ⭐ **FIXED**
   ```
   ❌ ROS2 not found. Please install ROS2 and source the setup.
   ```
   **Problem**: ROS2 environment variables not passed to Python subprocess
   **Solution**: Use the fixed launcher script
   ```bash
   ./start_test_publishers.sh  # Auto-sources ROS2 environment
   ```

6. **IMU Publisher Crashes** ⭐ **FIXED**
   ```
   AttributeError: module 'math' has no attribute 'random'
   ```
   **Problem**: Used `math.random()` instead of `random.random()`
   **Solution**: Fixed in `imu_test_publisher.py` - now uses correct `random.random()`

7. **Battery Dashboard Shows No Data** ⭐ **FIXED**
   ```
   Dashboard connects but no battery data appears
   ```
   **Problem**: Message type mismatch - publisher used `Float64`, dashboard expected `Float32`
   **Solution**: Fixed in `battery_test_publisher.py` - now uses `std_msgs/Float32` messages

8. **Publishers Start But Crash Immediately**
   ```
   🚀 Starting battery publisher...
   ❌ battery publisher failed to start
   ```
   **Solution**: Check the error output and ensure ROS2 environment is properly sourced
   ```bash
   # Use the automated launcher
   ./start_test_publishers.sh
   
   # Or manually source ROS2 first
   source /opt/ros/humble/setup.bash
   python3 battery_test_publisher.py
   ```

### Debugging Tips

1. **Check ROS2 Environment**
   ```bash
   echo $ROS_DOMAIN_ID
   ros2 doctor
   ```

2. **Verify Topic Publication**
   ```bash
   ros2 topic list | grep battery
   ros2 topic echo /battery/soc --once
   ```

3. **Monitor System Resources**
   ```bash
   htop  # Check CPU/memory usage
   ```

4. **Check Network Connectivity**
   ```bash
   netstat -tlnp | grep 9090  # ROS bridge port
   ```

## 📈 Performance Considerations

### Resource Usage

| Publisher | CPU Usage | Memory | Network | Frequency |
|-----------|-----------|---------|---------|-----------|
| Battery | Low | ~10MB | Low | 0.5 Hz |
| IMU | Low | ~15MB | Medium | 20 Hz |
| System | Low | ~20MB | Low | 0.5 Hz |
| Point Cloud | Medium | ~50MB | High | 10 Hz |
| Occupancy Map | Low | ~30MB | Medium | 0.2 Hz |
| Camera | High | ~100MB | Very High | 30 Hz |
| Robot Control | Low | ~25MB | Medium | 10 Hz |

### Optimization Tips

1. **Reduce Camera Resolution** (in camera_test_publisher.py):
   ```python
   self.image_width = 320  # Instead of 640
   self.image_height = 240  # Instead of 480
   ```

2. **Lower Point Cloud Density**:
   ```python
   self.point_count = 1000  # Instead of 5000
   ```

3. **Adjust Update Frequencies**:
   ```python
   self.timer = self.create_timer(0.1, callback)  # 10 Hz instead of higher
   ```

## 🔗 Integration with Dashboard

### Dashboard Routes

- **Main Dashboard**: `/` - 3D visualization with widgets
- **Battery Monitor**: `/battery` - Detailed battery analytics
- **System Monitor**: `/system-monitoring` - Brain system metrics
- **IMU Sensor**: `/imu-sensor` - Orientation and motion data
- **Robot Control**: `/control` - Joystick and navigation interface

### WebSocket Connection

The dashboard connects to ROS via WebSocket on `ws://localhost:9090`. Ensure rosbridge_server is running for proper communication.

### Real-time Updates

All publishers provide real-time data updates with appropriate frequencies for smooth dashboard operation:

- **High Frequency** (20-30 Hz): IMU, Camera feeds
- **Medium Frequency** (10 Hz): Point clouds, Robot control
- **Low Frequency** (0.2-2 Hz): Battery, System monitoring, Maps

## 📝 Customization

### Adding New Publishers

1. Create new publisher script following the existing pattern
2. Add entry to `run_all_test_publishers.py` publishers dictionary
3. Update this README with documentation
4. Test integration with dashboard

### Modifying Data Patterns

Each publisher includes configurable parameters for:
- Update frequencies
- Data ranges and variations
- Noise levels
- Scene complexity

Example customization in `battery_test_publisher.py`:
```python
# Modify these values for different battery behavior
self.base_soc = 85.0  # Starting charge level
self.total_capacity = 230.0  # Battery capacity in Ah
```

## 📚 Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [ROS Bridge Suite](http://wiki.ros.org/rosbridge_suite)
- [React Dashboard Documentation](../README.md)
- [System Architecture](../ENHANCED_SYSTEM_MONITORING.md)

## 🤝 Contributing

When adding new test publishers:

1. Follow the existing code structure and naming conventions
2. Include comprehensive error handling and logging
3. Add appropriate command-line arguments
4. Update the master launcher script
5. Document all topics and message formats
6. Test with the React dashboard

## 📄 License

This project is part of the ROS2 3D Point Cloud Dashboard system. See the main project license for details.
