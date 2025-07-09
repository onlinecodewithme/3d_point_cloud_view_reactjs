# ROS Bridge Integration Documentation

## Overview

This document provides comprehensive documentation for integrating each ROS topic with the ROS Bridge WebSocket server for the Point Cloud Visualization Dashboard. The dashboard uses `rosbridge_suite` to communicate with ROS2 topics via WebSocket connections.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [ROS Bridge Setup](#ros-bridge-setup)
3. [Point Cloud Topics](#point-cloud-topics)
4. [Battery System Topics](#battery-system-topics)
5. [Navigation Topics](#navigation-topics)
6. [SLAM/Mapping Topics](#slamapping-topics)
7. [Camera Topics](#camera-topics)
8. [Service Integration](#service-integration)
9. [Data Type Reference](#data-type-reference)
10. [Sample Data Examples](#sample-data-examples)
11. [Troubleshooting](#troubleshooting)

## Prerequisites

### Required ROS2 Packages
```bash
# Install rosbridge suite
sudo apt install ros-$ROS_DISTRO-rosbridge-suite

# Install web video server (for camera feeds)
sudo apt install ros-$ROS_DISTRO-web-video-server

# Install navigation stack
sudo apt install ros-$ROS_DISTRO-navigation2
sudo apt install ros-$ROS_DISTRO-nav2-bringup

# Install RTAB-Map
sudo apt install ros-$ROS_DISTRO-rtabmap-ros

# Install ZED SDK and ROS wrapper
# Follow ZED SDK installation guide for your system
```

### WebSocket Configuration
- **Default URL**: `ws://localhost:9090`
- **Protocol**: rosbridge WebSocket protocol v2.0
- **Auto-reconnect**: 3-second intervals
- **Connection Management**: Graceful handling of disconnections

## ROS Bridge Setup

### 1. Start ROS Bridge Server
```bash
# Terminal 1: Start rosbridge WebSocket server
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Optional: Custom port configuration
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

### 2. Start Web Video Server (for camera feeds)
```bash
# Terminal 2: Start web video server
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run web_video_server web_video_server
```

### 3. Verify Connection
```bash
# Check if rosbridge is running
ros2 node list | grep rosbridge

# Test WebSocket connection
curl -i -N -H "Connection: Upgrade" -H "Upgrade: websocket" -H "Sec-WebSocket-Key: test" -H "Sec-WebSocket-Version: 13" http://localhost:9090
```

## Point Cloud Topics

### Primary Point Cloud Topic

**Topic**: `/zed/zed_node/point_cloud/cloud_registered`
**Message Type**: `sensor_msgs/PointCloud2`
**Description**: Main 3D point cloud data from ZED camera with RGB colors

#### Data Structure
```typescript
interface PointCloud2Message {
  header: {
    stamp: {
      sec: number;
      nanosec: number;
    };
    frame_id: string;
  };
  height: number;        // Number of rows in point cloud
  width: number;         // Number of columns in point cloud
  fields: Array<{        // Field definitions
    name: string;        // 'x', 'y', 'z', 'rgb'
    offset: number;      // Byte offset in point data
    datatype: number;    // Data type (7=FLOAT32, 6=UINT32)
    count: number;       // Number of elements
  }>;
  is_bigendian: boolean;
  point_step: number;    // Bytes per point
  row_step: number;      // Bytes per row
  data: number[] | string; // Point cloud data (base64 or array)
  is_dense: boolean;
}
```

#### WebSocket Subscription
```javascript
const pointCloudTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/zed/zed_node/point_cloud/cloud_registered',
  messageType: 'sensor_msgs/PointCloud2'
});

pointCloudTopic.subscribe((message) => {
  // Process point cloud data
  console.log('Received point cloud:', message);
});
```

#### Sample Data
```json
{
  "header": {
    "stamp": {
      "sec": 1704067200,
      "nanosec": 123456789
    },
    "frame_id": "zed_left_camera_optical_frame"
  },
  "height": 376,
  "width": 672,
  "fields": [
    {"name": "x", "offset": 0, "datatype": 7, "count": 1},
    {"name": "y", "offset": 4, "datatype": 7, "count": 1},
    {"name": "z", "offset": 8, "datatype": 7, "count": 1},
    {"name": "rgb", "offset": 12, "datatype": 6, "count": 1}
  ],
  "is_bigendian": false,
  "point_step": 16,
  "row_step": 10752,
  "data": "base64_encoded_binary_data...",
  "is_dense": false
}
```

## Battery System Topics

### Battery State Topic

**Topic**: `/battery/state`
**Message Type**: `sensor_msgs/BatteryState`
**Description**: Comprehensive battery information

#### Data Structure
```typescript
interface BatteryState {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  voltage: number;           // Pack voltage (V)
  current: number;           // Current (A, negative=discharging)
  charge: number;            // Remaining charge (Ah)
  capacity: number;          // Total capacity (Ah)
  design_capacity: number;   // Design capacity (Ah)
  percentage: number;        // SOC (0.0-1.0)
  power_supply_status: number; // 0=Unknown, 1=Charging, 2=Discharging, 3=Not charging, 4=Full
  power_supply_health: number; // 1=Good, 2=Overheat, 3=Dead, 4=Overvoltage, 5=Unspecified failure
  power_supply_technology: number; // 0=Unknown, 1=NiMH, 2=LION, 3=LIPO, 4=LiFe, 5=NiCd, 6=LiMn
  present: boolean;
  cell_voltage: number[];    // Individual cell voltages
  location: string;
  serial_number: string;
}
```

#### WebSocket Subscription
```javascript
const batteryTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/battery/state',
  messageType: 'sensor_msgs/BatteryState'
});

batteryTopic.subscribe((message) => {
  console.log('Battery SOC:', message.percentage * 100 + '%');
  console.log('Battery voltage:', message.voltage + 'V');
  console.log('Battery current:', message.current + 'A');
});
```

### Individual Battery Topics

#### State of Charge
**Topic**: `/battery/soc`
**Message Type**: `std_msgs/Float32`
```javascript
const socTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/battery/soc',
  messageType: 'std_msgs/Float32'
});
```

#### Pack Voltage
**Topic**: `/battery/pack_voltage`
**Message Type**: `std_msgs/Float32`
```javascript
const voltageTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/battery/pack_voltage',
  messageType: 'std_msgs/Float32'
});
```

#### Current
**Topic**: `/battery/current`
**Message Type**: `std_msgs/Float32`
```javascript
const currentTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/battery/current',
  messageType: 'std_msgs/Float32'
});
```

#### Temperature
**Topic**: `/battery/temperature`
**Message Type**: `std_msgs/Float32MultiArray`
```javascript
const tempTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/battery/temperature',
  messageType: 'std_msgs/Float32MultiArray'
});
```

#### Cell Voltages
**Topic**: `/battery/cell_voltages`
**Message Type**: `std_msgs/Float32MultiArray`
```javascript
const cellTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/battery/cell_voltages',
  messageType: 'std_msgs/Float32MultiArray'
});
```

#### Sample Battery Data
```json
{
  "header": {
    "stamp": {"sec": 1704067200, "nanosec": 123456789},
    "frame_id": "battery"
  },
  "voltage": 24.6,
  "current": -2.3,
  "charge": 8.5,
  "capacity": 10.0,
  "percentage": 0.85,
  "power_supply_status": 2,
  "power_supply_health": 1,
  "cell_voltage": [4.1, 4.0, 4.1, 4.0, 4.1, 4.0],
  "present": true,
  "location": "main_battery",
  "serial_number": "BAT001"
}
```

## Navigation Topics

### Goal Publishing
**Topic**: `/move_base_simple/goal`
**Message Type**: `geometry_msgs/PoseStamped`
**Description**: Send navigation goals to the robot

#### Data Structure
```typescript
interface PoseStamped {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  pose: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
}
```

#### WebSocket Publishing
```javascript
const goalPublisher = new ROSLIB.Topic({
  ros: ros,
  name: '/move_base_simple/goal',
  messageType: 'geometry_msgs/PoseStamped'
});

const goal = {
  header: {
    frame_id: 'map',
    stamp: {
      sec: Math.floor(Date.now() / 1000),
      nanosec: (Date.now() % 1000) * 1000000
    }
  },
  pose: {
    position: { x: 2.0, y: 1.0, z: 0.0 },
    orientation: { x: 0, y: 0, z: 0, w: 1 }
  }
};

goalPublisher.publish(goal);
```

### Path Planning
**Topic**: `/plan`
**Message Type**: `nav_msgs/Path`
**Description**: Planned path from current position to goal

#### Data Structure
```typescript
interface Path {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  poses: Array<{
    header: {
      stamp: { sec: number; nanosec: number };
      frame_id: string;
    };
    pose: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  }>;
}
```

### Navigation Status
**Topic**: `/move_base/status`
**Message Type**: `actionlib_msgs/GoalStatusArray`
**Description**: Current navigation status

#### WebSocket Subscription
```javascript
const statusTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/move_base/status',
  messageType: 'actionlib_msgs/GoalStatusArray'
});

statusTopic.subscribe((message) => {
  const hasActiveGoal = message.status_list.some(status => status.status === 1);
  console.log('Navigation active:', hasActiveGoal);
});
```

### Goal Cancellation
**Topic**: `/move_base/cancel`
**Message Type**: `actionlib_msgs/GoalID`
```javascript
const cancelPublisher = new ROSLIB.Topic({
  ros: ros,
  name: '/move_base/cancel',
  messageType: 'actionlib_msgs/GoalID'
});

cancelPublisher.publish({}); // Cancel all goals
```

## SLAM/Mapping Topics

### Occupancy Grid Map
**Topic**: `/map`
**Message Type**: `nav_msgs/OccupancyGrid`
**Description**: 2D occupancy grid map from SLAM

#### Data Structure
```typescript
interface OccupancyGrid {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  info: {
    map_load_time: { sec: number; nanosec: number };
    resolution: number;    // meters per pixel
    width: number;         // map width in pixels
    height: number;        // map height in pixels
    origin: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
  data: number[];         // occupancy data: -1=unknown, 0-100=occupancy probability
}
```

#### WebSocket Subscription
```javascript
const mapTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/map',
  messageType: 'nav_msgs/OccupancyGrid'
});

mapTopic.subscribe((message) => {
  console.log('Map size:', message.info.width, 'x', message.info.height);
  console.log('Resolution:', message.info.resolution, 'm/pixel');
});
```

#### Sample Map Data
```json
{
  "header": {
    "stamp": {"sec": 1704067200, "nanosec": 123456789},
    "frame_id": "map"
  },
  "info": {
    "resolution": 0.05,
    "width": 384,
    "height": 384,
    "origin": {
      "position": {"x": -10.0, "y": -10.0, "z": 0.0},
      "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
    }
  },
  "data": [0, 0, 0, 100, 100, -1, -1, ...]
}
```

### RTAB-Map Topics

#### Map Data
**Topic**: `/rtabmap/mapData`
**Message Type**: `rtabmap_ros/MapData`

#### Grid Map
**Topic**: `/rtabmap/grid_map`
**Message Type**: `nav_msgs/OccupancyGrid`

#### Point Cloud Map
**Topic**: `/rtabmap/cloud_map`
**Message Type**: `sensor_msgs/PointCloud2`

#### Loop Closure Detection
**Topic**: `/rtabmap/info`
**Message Type**: `rtabmap_ros/Info`

## Camera Topics

### RGB Image Feed
**Topic**: `/zed/zed_node/rgb/image_rect_color`
**Message Type**: `sensor_msgs/Image`
**Description**: Rectified RGB image from ZED camera

#### Data Structure
```typescript
interface Image {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  height: number;        // image height in pixels
  width: number;         // image width in pixels
  encoding: string;      // 'rgb8', 'bgr8', 'rgba8', 'mono8'
  is_bigendian: boolean;
  step: number;          // bytes per row
  data: number[] | string; // image data (base64 or array)
}
```

#### WebSocket Subscription
```javascript
const imageTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/zed/zed_node/rgb/image_rect_color',
  messageType: 'sensor_msgs/Image'
});

imageTopic.subscribe((message) => {
  console.log('Image:', message.width, 'x', message.height, message.encoding);
  // Process image data...
});
```

### Alternative: Web Video Server
For better performance, use Web Video Server for camera feeds:
```
http://localhost:8080/stream?topic=/zed/zed_node/rgb/image_rect_color&type=mjpeg
```

### Depth Image
**Topic**: `/zed/zed_node/depth/depth_registered`
**Message Type**: `sensor_msgs/Image`

### Camera Info
**Topic**: `/zed/zed_node/rgb/camera_info`
**Message Type**: `sensor_msgs/CameraInfo`

## Service Integration

### Map Saving Service
**Service**: `/save_map`
**Service Type**: `nav2_msgs/SaveMap`

#### Service Call
```javascript
const saveMapService = new ROSLIB.Service({
  ros: ros,
  name: '/save_map',
  serviceType: 'nav2_msgs/SaveMap'
});

const request = new ROSLIB.ServiceRequest({
  map_topic: '/map',
  map_url: '/tmp/my_map',
  image_format: 'pgm',
  map_mode: 'trinary',
  free_thresh: 0.25,
  occupied_thresh: 0.65
});

saveMapService.callService(request, (result) => {
  console.log('Map save result:', result.result);
});
```

### Topic List Service
**Service**: `/rosapi/topics`
**Service Type**: `rosapi/Topics`

```javascript
const topicsService = new ROSLIB.Service({
  ros: ros,
  name: '/rosapi/topics',
  serviceType: 'rosapi/Topics'
});

topicsService.callService({}, (result) => {
  console.log('Available topics:', result.topics);
});
```

## Data Type Reference

### Standard Message Types

#### std_msgs/Float32
```json
{
  "data": 42.5
}
```

#### std_msgs/String
```json
{
  "data": "Hello World"
}
```

#### std_msgs/Float32MultiArray
```json
{
  "layout": {
    "dim": [{"label": "cells", "size": 6, "stride": 6}],
    "data_offset": 0
  },
  "data": [4.1, 4.0, 4.1, 4.0, 4.1, 4.0]
}
```

#### geometry_msgs/Point
```json
{
  "x": 1.0,
  "y": 2.0,
  "z": 3.0
}
```

#### geometry_msgs/Quaternion
```json
{
  "x": 0.0,
  "y": 0.0,
  "z": 0.0,
  "w": 1.0
}
```

### Custom Message Types

#### rtabmap_ros/Info
Contains SLAM statistics and loop closure information.

#### rtabmap_ros/MapData
Contains complete RTAB-Map data including nodes, links, and signatures.

## Sample Data Examples

### Complete Point Cloud Message
```json
{
  "topic": "/zed/zed_node/point_cloud/cloud_registered",
  "msg": {
    "header": {
      "stamp": {"sec": 1704067200, "nanosec": 123456789},
      "frame_id": "zed_left_camera_optical_frame"
    },
    "height": 376,
    "width": 672,
    "fields": [
      {"name": "x", "offset": 0, "datatype": 7, "count": 1},
      {"name": "y", "offset": 4, "datatype": 7, "count": 1},
      {"name": "z", "offset": 8, "datatype": 7, "count": 1},
      {"name": "rgb", "offset": 12, "datatype": 6, "count": 1}
    ],
    "is_bigendian": false,
    "point_step": 16,
    "row_step": 10752,
    "data": "iVBORw0KGgoAAAANSUhEUgAA...",
    "is_dense": false
  },
  "op": "publish"
}
```

### Complete Battery Message
```json
{
  "topic": "/battery/state",
  "msg": {
    "header": {
      "stamp": {"sec": 1704067200, "nanosec": 123456789},
      "frame_id": "battery"
    },
    "voltage": 24.6,
    "current": -2.3,
    "charge": 8.5,
    "capacity": 10.0,
    "design_capacity": 10.0,
    "percentage": 0.85,
    "power_supply_status": 2,
    "power_supply_health": 1,
    "power_supply_technology": 2,
    "present": true,
    "cell_voltage": [4.1, 4.0, 4.1, 4.0, 4.1, 4.0],
    "location": "main_battery",
    "serial_number": "BAT001"
  },
  "op": "publish"
}
```

### Complete Navigation Goal
```json
{
  "topic": "/move_base_simple/goal",
  "msg": {
    "header": {
      "stamp": {"sec": 1704067200, "nanosec": 123456789},
      "frame_id": "map"
    },
    "pose": {
      "position": {"x": 2.0, "y": 1.0, "z": 0.0},
      "orientation": {"x": 0, "y": 0, "z": 0.7071, "w": 0.7071}
    }
  },
  "op": "publish"
}
```

## Troubleshooting

### Common Issues

#### 1. WebSocket Connection Failed
```bash
# Check if rosbridge is running
ros2 node list | grep rosbridge

# Check port availability
netstat -an | grep 9090

# Restart rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

#### 2. No Point Cloud Data
```bash
# Check if ZED camera is publishing
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered

# Check topic list
ros2 topic list | grep zed

# Check ZED camera node
ros2 node list | grep zed
```

#### 3. Battery Data Not Available
```bash
# Check battery topics
ros2 topic list | grep battery

# Test battery publisher
python3 battery_test_publisher.py

# Check message type
ros2 topic info /battery/state
```

#### 4. Navigation Not Working
```bash
# Check navigation stack
ros2 node list | grep nav

# Check map topic
ros2 topic hz /map

# Verify goal topic
ros2 topic info /move_base_simple/goal
```

#### 5. Camera Feed Issues
```bash
# Check web video server
ros2 run web_video_server web_video_server

# Test camera topic
ros2 topic hz /zed/zed_node/rgb/image_rect_color

# Check image encoding
ros2 topic echo /zed/zed_node/rgb/image_rect_color --field encoding
```

### Debug Commands

#### Monitor Topic Activity
```bash
# List all topics
ros2 topic list

# Check topic frequency
ros2 topic hz <topic_name>

# Echo topic data
ros2 topic echo <topic_name>

# Get topic info
ros2 topic info <topic_name>
```

#### WebSocket Debug
```javascript
// Enable debug logging in browser console
const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

ros.on('connection', () => {
  console.log('Connected to websocket server.');
});

ros.on('error', (error) => {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', () => {
  console.log('Connection to websocket server closed.');
});
```

### Performance Optimization

#### 1. Point Cloud Optimization
- Reduce point cloud resolution in ZED configuration
- Use voxel filtering to reduce point count
- Implement level-of-detail (LOD) rendering

#### 2. Battery Monitoring
- Reduce update frequency for non-critical data
- Use message throttling for high-frequency topics
- Implement data caching

#### 3. Navigation Optimization
- Use compressed image transport
- Implement path smoothing
- Cache waypoint data locally

#### 4. WebSocket Optimization
- Enable compression in rosbridge
- Use binary transport for large messages
- Implement connection pooling

### Configuration Files

#### rosbridge_websocket_launch.xml
```xml
<launch>
  <node name="rosbridge_websocket" pkg="rosbridge_server" type="rosbridge_websocket" output="screen">
    <param name="port" value="9090"/>
    <param name="address" value=""/>
    <param name="retry_startup_delay" value="5"/>
    <param name="fragment_timeout" value="600"/>
    <param name="delay_between_messages" value="0"/>
    <param name="max_message_size" value="None"/>
    <param name="unregister_timeout" value="10"/>
  </node>
</launch>
```

#### Web Video Server Configuration
```bash
# Start with custom parameters
ros2 run web_video_server web_video_server \
  --ros-args \
  -p port:=8080 \
  -p address:=0.0.0.0 \
  -p default_stream_type:=mjpeg
```

This documentation provides a complete reference for integrating all ROS topics with the ROS Bridge WebSocket server. Each topic includes data structures, sample code, and troubleshooting information to ensure successful integration.
