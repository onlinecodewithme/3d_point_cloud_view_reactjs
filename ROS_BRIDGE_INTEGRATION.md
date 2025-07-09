# ROS Bridge Integration Documentation

## Overview

This document provides comprehensive documentation for integrating each ROS topic with the ROS Bridge WebSocket server for the Point Cloud Visualization Dashboard. The dashboard uses `rosbridge_suite` to communicate with ROS2 topics via WebSocket connections.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [ROS Bridge Setup](#ros-bridge-setup)
3. [Point Cloud Topics](#point-cloud-topics)
4. [Battery System Topics](#battery-system-topics)
5. [System Monitoring Topics](#system-monitoring-topics)
6. [Navigation Topics](#navigation-topics)
7. [SLAM/Mapping Topics](#slamapping-topics)
8. [Camera Topics](#camera-topics)
9. [Service Integration](#service-integration)
10. [Data Type Reference](#data-type-reference)
11. [Sample Data Examples](#sample-data-examples)
12. [Troubleshooting](#troubleshooting)

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

## System Monitoring Topics

### Enhanced Brain System Monitoring

The Enhanced Brain System Monitoring Dashboard subscribes to dedicated system monitoring topics for both Cognition and Perception Brain systems. These topics provide comprehensive system health and performance data.

#### Cognition Brain System Monitoring

**Topic**: `/cognitionbrain/system_monitoring`
**Message Type**: `std_msgs/String`
**Description**: Comprehensive system monitoring data from the Cognition Brain system in JSON format

##### Data Structure
```typescript
interface CognitionBrainSystemData {
  timestamp: number;
  header: {
    frame_id: string;
    timestamp: number;
  };
  cpu: {
    usage_percent: number;
    temperature: number;
    core_count: number;
    core_usage: number[];
    frequency_mhz: number;
  };
  memory: {
    total_bytes: number;
    used_bytes: number;
    available_bytes: number;
    usage_percent: number;
  };
  gpu: {
    available: boolean;
    usage_percent: number;
    temperature: number;
    memory_total_bytes: number;
    memory_used_bytes: number;
    memory_usage_percent: number;
  };
  storage: {
    total_bytes: number;
    used_bytes: number;
    available_bytes: number;
    usage_percent: number;
  };
  network: {
    bytes_sent: number;
    bytes_received: number;
    packets_sent: number;
    packets_received: number;
  };
  system_load: {
    load_1min: number;
    load_5min: number;
    load_15min: number;
  };
  processes: {
    process_count: number;
    thread_count: number;
  };
  uptime_seconds: number;
  battery: {
    available: boolean;
    percent: number;
    charging: boolean;
    time_remaining_minutes: number;
  };
}
```

##### WebSocket Subscription
```javascript
const cognitionTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/cognitionbrain/system_monitoring',
  messageType: 'std_msgs/String'
});

cognitionTopic.subscribe((message) => {
  try {
    const systemData = JSON.parse(message.data);
    console.log('Cognition Brain CPU:', systemData.cpu.usage_percent + '%');
    console.log('Cognition Brain Memory:', systemData.memory.usage_percent + '%');
    console.log('Cognition Brain Temperature:', systemData.cpu.temperature + '°C');
    
    // Process system monitoring data
    updateCognitionBrainMetrics(systemData);
  } catch (error) {
    console.error('Error parsing cognition brain data:', error);
  }
});
```

##### Sample Cognition Brain Data
```json
{
  "data": "{\"timestamp\":1752077026.528329,\"header\":{\"frame_id\":\"system_monitor\",\"timestamp\":1752077026.5283687},\"cpu\":{\"usage_percent\":2.5,\"temperature\":0.0,\"core_count\":8,\"core_usage\":[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],\"frequency_mhz\":729.6},\"memory\":{\"total_bytes\":16416145408,\"used_bytes\":4179402752,\"available_bytes\":11845648384,\"usage_percent\":27.8},\"gpu\":{\"available\":true,\"usage_percent\":0.0,\"temperature\":0.0,\"memory_total_bytes\":0,\"memory_used_bytes\":0,\"memory_usage_percent\":0.0},\"storage\":{\"total_bytes\":123887923200,\"used_bytes\":91493199872,\"available_bytes\":26054246400,\"usage_percent\":73.85},\"network\":{\"bytes_sent\":834550655,\"bytes_received\":912503764,\"packets_sent\":1525461,\"packets_received\":1611076},\"system_load\":{\"load_1min\":0.04931640625,\"load_5min\":0.2529296875,\"load_15min\":0.20068359375},\"processes\":{\"process_count\":359,\"thread_count\":753},\"uptime_seconds\":45610,\"battery\":{\"available\":false,\"percent\":0.0,\"charging\":false,\"time_remaining_minutes\":0}}"
}
```

#### Perception Brain System Monitoring

**Topic**: `/perceptionbrain/system_monitoring`
**Message Type**: `std_msgs/String`
**Description**: Comprehensive system monitoring data from the Perception Brain system in JSON format with extended system information

##### Data Structure
```typescript
interface PerceptionBrainSystemData {
  header: {
    timestamp: string;
    frame_id: string;
    node_name: string;
  };
  cpu: {
    usage_percent: string;
    temperature: string;
    core_count: string;
    core_usage: string; // Comma-separated values
    frequency_mhz: string;
  };
  memory: {
    total_bytes: string;
    used_bytes: string;
    available_bytes: string;
    usage_percent: string;
    total_gb: string;
    used_gb: string;
    available_gb: string;
  };
  gpu: {
    available: string;
    usage_percent: string;
    temperature: string;
    memory_total_bytes: string;
    memory_used_bytes: string;
    memory_usage_percent: string;
    memory_total_gb: string;
    memory_used_gb: string;
  };
  storage: {
    total_bytes: string;
    used_bytes: string;
    available_bytes: string;
    usage_percent: string;
    path: string;
    total_gb: string;
    used_gb: string;
    available_gb: string;
  };
  network: {
    bytes_sent: string;
    bytes_received: string;
    packets_sent: string;
    packets_received: string;
    bytes_sent_mb: string;
    bytes_received_mb: string;
  };
  system_load: {
    load_1min: string;
    load_5min: string;
    load_15min: string;
  };
  processes: {
    process_count: string;
    thread_count: string;
  };
  uptime: {
    seconds: string;
    formatted: string;
    days: string;
    hours: string;
    minutes: string;
  };
  battery: {
    available: string;
    percent: string;
    charging: string;
    time_remaining_minutes: string;
    time_remaining_hours: string;
  };
  system_info: {
    platform: string;
    platform_release: string;
    platform_version: string;
    architecture: string;
    hostname: string;
    processor: string;
  };
}
```

##### WebSocket Subscription
```javascript
const perceptionTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/perceptionbrain/system_monitoring',
  messageType: 'std_msgs/String'
});

perceptionTopic.subscribe((message) => {
  try {
    const systemData = JSON.parse(message.data);
    
    // Parse string values to numbers
    const parseValue = (value) => {
      if (typeof value === 'string') {
        const parsed = parseFloat(value);
        return isNaN(parsed) ? 0 : parsed;
      }
      return typeof value === 'number' ? value : 0;
    };
    
    console.log('Perception Brain CPU:', parseValue(systemData.cpu.usage_percent) + '%');
    console.log('Perception Brain Memory:', parseValue(systemData.memory.usage_percent) + '%');
    console.log('Perception Brain Hostname:', systemData.system_info.hostname);
    
    // Process system monitoring data
    updatePerceptionBrainMetrics(systemData);
  } catch (error) {
    console.error('Error parsing perception brain data:', error);
  }
});
```

##### Sample Perception Brain Data
```json
{
  "data": "{\"header\":{\"timestamp\":\"1752077026.589248\",\"frame_id\":\"system_monitor\",\"node_name\":\"perception_brain_system_monitor\"},\"cpu\":{\"usage_percent\":\"2.5\",\"temperature\":\"0.0\",\"core_count\":\"8\",\"core_usage\":\"0.0,0.0,0.0,0.0,9.1,0.0,0.0,9.1\",\"frequency_mhz\":\"768.0\"},\"memory\":{\"total_bytes\":\"16415924224\",\"used_bytes\":\"3798114304\",\"available_bytes\":\"12267659264\",\"usage_percent\":\"25.3\",\"total_gb\":\"15.3\",\"used_gb\":\"3.5\",\"available_gb\":\"11.4\"},\"gpu\":{\"available\":\"true\",\"usage_percent\":\"0.0\",\"temperature\":\"0.0\",\"memory_total_bytes\":\"0\",\"memory_used_bytes\":\"0\",\"memory_usage_percent\":\"0.0\",\"memory_total_gb\":\"0.0\",\"memory_used_gb\":\"0.0\"},\"storage\":{\"total_bytes\":\"1005442326528\",\"used_bytes\":\"35824812032\",\"available_bytes\":\"918468345856\",\"usage_percent\":\"3.6\",\"path\":\"/\",\"total_gb\":\"936.4\",\"used_gb\":\"33.4\",\"available_gb\":\"855.4\"},\"network\":{\"bytes_sent\":\"1887307664\",\"bytes_received\":\"1774041922\",\"packets_sent\":\"13786580\",\"packets_received\":\"13760789\",\"bytes_sent_mb\":\"1799.9\",\"bytes_received_mb\":\"1691.9\"},\"system_load\":{\"load_1min\":\"0.64\",\"load_5min\":\"0.69\",\"load_15min\":\"0.78\"},\"processes\":{\"process_count\":\"399\",\"thread_count\":\"929\"},\"uptime\":{\"seconds\":\"46504\",\"formatted\":\"12h 55m 4s\",\"days\":\"0\",\"hours\":\"12\",\"minutes\":\"55\"},\"battery\":{\"available\":\"false\",\"percent\":\"0.0\",\"charging\":\"false\",\"time_remaining_minutes\":\"0\",\"time_remaining_hours\":\"0.0\"},\"system_info\":{\"platform\":\"Linux\",\"platform_release\":\"5.15.136-tegra\",\"platform_version\":\"#1 SMP PREEMPT Mon May 6 09:56:39 PDT 2024\",\"architecture\":\"aarch64\",\"hostname\":\"xavier-desktop\",\"processor\":\"aarch64\"}}"
}
```

#### System Monitoring Integration Example

```javascript
class EnhancedSystemMonitoring {
  constructor(ros) {
    this.ros = ros;
    this.systemData = {};
    this.historicalData = [];
    this.setupSubscriptions();
  }

  setupSubscriptions() {
    // Subscribe to both brain systems
    this.subscribeToCognitionBrain();
    this.subscribeToPerceptionBrain();
  }

  subscribeToCognitionBrain() {
    const cognitionTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/cognitionbrain/system_monitoring',
      messageType: 'std_msgs/String'
    });

    cognitionTopic.subscribe((message) => {
      try {
        const data = JSON.parse(message.data);
        this.processSystemData(data, 'Cognition Brain', 'cognition');
      } catch (error) {
        console.error('Error processing cognition brain data:', error);
      }
    });
  }

  subscribeToPerceptionBrain() {
    const perceptionTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/perceptionbrain/system_monitoring',
      messageType: 'std_msgs/String'
    });

    perceptionTopic.subscribe((message) => {
      try {
        const data = JSON.parse(message.data);
        this.processSystemData(data, 'Perception Brain', 'perception');
      } catch (error) {
        console.error('Error processing perception brain data:', error);
      }
    });
  }

  processSystemData(data, brainName, brainKey) {
    // Normalize data format differences between brain systems
    const parseValue = (value) => {
      if (typeof value === 'string') {
        const parsed = parseFloat(value);
        return isNaN(parsed) ? 0 : parsed;
      }
      return typeof value === 'number' ? value : 0;
    };

    const normalizedMetrics = {
      timestamp: Date.now(),
      cpuUsage: parseValue(data.cpu?.usage || data.cpu?.usage_percent),
      memoryUsage: parseValue(data.memory?.usage || data.memory?.usage_percent),
      diskUsage: parseValue(data.disk?.usage || data.storage?.usage_percent),
      temperature: parseValue(data.cpu?.temperature || data.thermal?.temperature),
      networkRx: parseValue(data.network?.bytes_received),
      networkTx: parseValue(data.network?.bytes_sent),
      processCount: parseValue(data.processes?.process_count),
      uptime: parseValue(data.uptime_seconds || data.uptime?.seconds),
      loadAverage: [
        parseValue(data.system_load?.load_1min),
        parseValue(data.system_load?.load_5min),
        parseValue(data.system_load?.load_15min)
      ]
    };

    // Store normalized data
    this.systemData[brainKey] = {
      brainName,
      metrics: normalizedMetrics,
      isOnline: true,
      lastUpdate: Date.now()
    };

    // Add to historical data for charts
    this.addHistoricalDataPoint();
    
    // Update dashboard
    this.updateDashboard();
  }

  addHistoricalDataPoint() {
    const now = Date.now();
    const timeString = new Date(now).toLocaleTimeString();
    
    const cognitionData = this.systemData['cognition'];
    const perceptionData = this.systemData['perception'];
    
    if (cognitionData || perceptionData) {
      const dataPoint = {
        timestamp: now,
        time: timeString,
        cognitionCpu: cognitionData?.metrics.cpuUsage || 0,
        perceptionCpu: perceptionData?.metrics.cpuUsage || 0,
        cognitionMemory: cognitionData?.metrics.memoryUsage || 0,
        perceptionMemory: perceptionData?.metrics.memoryUsage || 0,
        cognitionTemp: cognitionData?.metrics.temperature || 0,
        perceptionTemp: perceptionData?.metrics.temperature || 0
      };
      
      this.historicalData.push(dataPoint);
      
      // Keep only last 60 data points (1 minute at 1-second intervals)
      if (this.historicalData.length > 60) {
        this.historicalData.shift();
      }
    }
  }

  updateDashboard() {
    // Update charts and UI components
    console.log('System data updated:', this.systemData);
    console.log('Historical data points:', this.historicalData.length);
  }
}

// Usage
const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
const systemMonitoring = new EnhancedSystemMonitoring(ros);
```

#### System Monitoring Troubleshooting

##### Common Issues

###### 1. No System Monitoring Data
```bash
# Check if system monitoring topics exist
ros2 topic list | grep system_monitoring

# Check if nodes are publishing
ros2 topic hz /cognitionbrain/system_monitoring
ros2 topic hz /perceptionbrain/system_monitoring

# Echo topic data to verify format
ros2 topic echo /cognitionbrain/system_monitoring --field data
```

###### 2. JSON Parsing Errors
```javascript
// Add error handling for malformed JSON
cognitionTopic.subscribe((message) => {
  try {
    const data = JSON.parse(message.data);
    // Process data...
  } catch (error) {
    console.error('JSON parsing error:', error);
    console.log('Raw message:', message.data);
  }
});
```

###### 3. Data Format Inconsistencies
```javascript
// Robust value parsing function
const parseValue = (value) => {
  if (value === null || value === undefined) return 0;
  if (typeof value === 'string') {
    // Handle comma-separated values (like core_usage)
    if (value.includes(',')) {
      return value.split(',').map(v => parseFloat(v.trim()) || 0);
    }
    const parsed = parseFloat(value);
    return isNaN(parsed) ? 0 : parsed;
  }
  return typeof value === 'number' ? value : 0;
};
```

###### 4. Performance Issues with High-Frequency Data
```javascript
// Implement data throttling
let lastUpdateTime = 0;
const UPDATE_INTERVAL = 1000; // 1 second

cognitionTopic.subscribe((message) => {
  const now = Date.now();
  if (now - lastUpdateTime < UPDATE_INTERVAL) {
    return; // Skip this update
  }
  lastUpdateTime = now;
  
  // Process data...
});
```

##### Debug Commands for System Monitoring

```bash
# Check system monitoring node status
ros2 node list | grep system_monitor

# Verify topic message types
ros2 topic info /cognitionbrain/system_monitoring
ros2 topic info /perceptionbrain/system_monitoring

# Monitor topic frequency
ros2 topic hz /cognitionbrain/system_monitoring
ros2 topic hz /perceptionbrain/system_monitoring

# Check for any errors in topic data
ros2 topic echo /cognitionbrain/system_monitoring | head -5
ros2 topic echo /perceptionbrain/system_monitoring | head -5
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
