# Robot Status Dashboard

A comprehensive React dashboard for real-time robot status monitoring, designed for robotics applications with multiple operational states and system health tracking.

## Overview

The Robot Status Dashboard provides comprehensive monitoring of robot operational status with the following key states:

- **🟦 IDLE** - Robot is idle and waiting for commands
- **🟢 READY** - Robot is ready to execute tasks
- **🟡 RUNNING** - Robot is actively executing a task
- **🟠 NOT RESPONDING** - Robot is not responding to commands
- **🔴 ERROR** - Robot has encountered an error

## Features

### 🤖 Robot Status Monitoring
- **Large status circle indicator** with color-coded states
- **Real-time status transitions** with appropriate animations
- **Current task display** with progress tracking
- **Uptime monitoring** with formatted display
- **Error message display** for troubleshooting

### 🔧 System Health Monitoring
- **CPU Usage** - Real-time processor utilization
- **Memory Usage** - RAM consumption monitoring
- **System Temperature** - Thermal monitoring with safety thresholds
- **Disk Usage** - Storage space monitoring
- **Color-coded health bars** with warning levels

### ⚙️ Subsystem Status
- **Navigation System** - Path planning and movement control
- **Perception System** - Sensor data processing and environment understanding
- **Manipulation System** - Robotic arm and gripper control
- **Communication System** - Network and ROS connectivity

### 📶 Connectivity & Location
- **ROS Connection Status** - Real-time bridge connectivity
- **Network Latency** - Communication delay monitoring
- **Signal Strength** - Wireless connection quality
- **Robot Position** - X, Y coordinates and orientation
- **Battery Level** - Power status with color-coded warnings

### 🚨 Emergency Features
- **Emergency Stop Detection** - Critical safety alert display
- **Heartbeat Monitoring** - Robot responsiveness tracking
- **Critical Status Animations** - Visual alerts for urgent conditions
- **Not Responding Warnings** - Connection loss indicators

## Technical Specifications

### ROS Topic Integration

#### Robot Status Topics
```
/robot/status                 - std_msgs/String (idle|ready|running|not_responding|error)
/robot/heartbeat             - std_msgs/Header (periodic heartbeat signal)
/robot/current_task          - std_msgs/String (current task description)
/robot/task_progress         - std_msgs/Float32 (task completion percentage)
/robot/system_health         - diagnostic_msgs/DiagnosticArray (system diagnostics)
```

#### System Health Data Structure
```typescript
interface RobotStatus {
  timestamp: number;
  status: 'idle' | 'ready' | 'running' | 'not_responding' | 'error';
  lastHeartbeat: number;
  uptime: number;
  currentTask: string;
  taskProgress: number;
  errorMessage?: string;
  systemHealth: {
    cpu: number;
    memory: number;
    temperature: number;
    diskSpace: number;
  };
  connectivity: {
    rosConnection: boolean;
    networkLatency: number;
    signalStrength: number;
  };
  subsystems: {
    navigation: 'online' | 'offline' | 'error';
    perception: 'online' | 'offline' | 'error';
    manipulation: 'online' | 'offline' | 'error';
    communication: 'online' | 'offline' | 'error';
  };
  location: {
    x: number;
    y: number;
    z: number;
    orientation: number;
  };
  batteryLevel: number;
  emergencyStop: boolean;
}
```

### Status State Machine

#### State Transitions
```
IDLE → READY, ERROR
READY → RUNNING, IDLE, ERROR
RUNNING → READY, ERROR, NOT_RESPONDING
NOT_RESPONDING → IDLE, ERROR
ERROR → IDLE, READY
```

#### Status Priorities
1. **IDLE** (Priority 1) - Normal standby state
2. **READY** (Priority 2) - Prepared for operation
3. **RUNNING** (Priority 3) - Active operation
4. **NOT_RESPONDING** (Priority 4) - Communication issue
5. **ERROR** (Priority 5) - Critical failure

### Health Monitoring Thresholds

#### System Health Limits
- **CPU Usage**: Good < 50%, Warning 50-80%, Critical > 80%
- **Memory Usage**: Good < 50%, Warning 50-80%, Critical > 80%
- **Temperature**: Good < 60°C, Warning 60-80°C, Critical > 80°C
- **Disk Usage**: Good < 50%, Warning 50-80%, Critical > 80%

#### Battery Levels
- **Good**: > 50% (Green)
- **Warning**: 20-50% (Orange)
- **Critical**: < 20% (Red)

#### Connectivity Thresholds
- **Network Latency**: Good < 50ms, Warning 50-100ms, Poor > 100ms
- **Signal Strength**: Excellent > 75%, Good 50-75%, Poor < 50%

## Dashboard Features

### Visual Design
- **Futuristic HUD-style interface** with holographic effects
- **Animated status circles** with state-specific animations
- **Color-coded system health** indicators
- **Responsive grid layout** for different screen sizes
- **Real-time data animations** and smooth transitions

### Status Animations
- **IDLE**: Gentle pulsing blue animation
- **READY**: Steady green pulse animation
- **RUNNING**: Continuous rotation animation
- **NOT_RESPONDING**: Blinking orange animation
- **ERROR**: Rapid red pulsing animation

### Interactive Elements
- **Hover effects** on status cards
- **Test data mode** for development and demonstration
- **Real-time connection status** indicator
- **Emergency stop alerts** with prominent warnings

### Performance Optimizations
- **Efficient React hooks** for state management
- **Optimized re-rendering** with proper dependencies
- **Smooth CSS animations** with hardware acceleration
- **Responsive design** for mobile and desktop

## Usage

### Accessing the Dashboard
Navigate to: `http://localhost:3000/robot-status`

### Test Data Mode
The dashboard includes a built-in test data generator:
1. Click "🧪 Use Test Data" button
2. Realistic robot status data will be generated automatically
3. Status changes every 15-30 seconds for demonstration
4. Click "⏹️ Stop Test Data" to return to ROS mode

### ROS Integration
The dashboard automatically connects to ROS topics via rosbridge:
- Ensure rosbridge_server is running: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Topics are subscribed automatically on connection
- Real-time data updates every 2-3 seconds
- Heartbeat monitoring with 10-second timeout

## Development

### File Structure
```
src/components/
├── RobotStatusDashboard.tsx    # Main dashboard component
├── RobotStatusDashboard.css    # Styling and animations
test_publishers/
├── robot_status_test_publisher.py  # Test data publisher
```

### Key Components

#### Status Configuration
```typescript
const statusConfigs: Record<string, StatusConfig> = {
  idle: {
    color: '#00aaff',
    icon: '🟦',
    description: 'Robot is idle and waiting for commands',
    priority: 1
  },
  // ... other status configurations
};
```

#### Health Color Coding
```typescript
const getHealthColor = (value: number, type: string) => {
  switch (type) {
    case 'cpu':
    case 'memory':
    case 'disk':
      if (value < 50) return '#00ff41';
      if (value < 80) return '#ffaa00';
      return '#ff4444';
    case 'temperature':
      if (value < 60) return '#00ff41';
      if (value < 80) return '#ffaa00';
      return '#ff4444';
  }
};
```

### Testing

#### Running Test Publisher
```bash
# Individual robot status publisher
python3 test_publishers/robot_status_test_publisher.py

# All publishers including robot status
python3 test_publishers/run_all_test_publishers.py

# Only robot status
python3 test_publishers/run_all_test_publishers.py --publishers robot_status
```

#### Test Data Features
- **Realistic status transitions** following state machine logic
- **Dynamic task scenarios** based on current status
- **System health variations** correlated with robot status
- **Emergency stop simulation** for critical testing
- **Heartbeat simulation** with timeout scenarios

## Troubleshooting

### Common Issues

#### No Status Data Displayed
1. Check ROS bridge connection: `ros2 topic list`
2. Verify robot status topics are published
3. Check browser console for connection errors
4. Use test data mode to verify dashboard functionality

#### Incorrect Status Display
1. Verify topic message types match expected format
2. Check robot status publisher implementation
3. Monitor ROS topics: `ros2 topic echo /robot/status`
4. Validate status state transitions

#### Performance Issues
1. Reduce update frequency in test publisher
2. Check browser performance in developer tools
3. Ensure adequate system resources
4. Consider reducing animation complexity

### Debug Mode
Enable debug logging by opening browser developer console:
- Connection status messages
- Received robot status data
- Status transition logs
- System health updates
- Error messages and warnings

## Integration with Robot Systems

### ROS2 Integration
```python
# Example robot status publisher
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')
        self.status_pub = self.create_publisher(String, '/robot/status', 10)
        
    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
```

### System Health Integration
```python
# Example system health publisher
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

def publish_system_health(self):
    health_msg = DiagnosticArray()
    
    # CPU status
    cpu_status = DiagnosticStatus()
    cpu_status.name = 'CPU Usage'
    cpu_status.message = str(cpu_usage)
    cpu_status.level = DiagnosticStatus.OK  # or WARN, ERROR
    
    health_msg.status = [cpu_status]
    self.health_pub.publish(health_msg)
```

## Future Enhancements

### Planned Features
- **Historical status logging** and trend analysis
- **Alert system** with email/SMS notifications
- **Custom status states** for specific robot applications
- **Multi-robot monitoring** for fleet management
- **Status prediction** using machine learning
- **Integration with robot planning systems**

### Advanced Monitoring
- **Performance metrics** tracking
- **Fault prediction** algorithms
- **Maintenance scheduling** based on system health
- **Remote diagnostics** capabilities
- **Status reporting** and analytics

## Contributing

When extending the Robot Status Dashboard:

1. **Update the RobotStatus interface** with new fields
2. **Add ROS topic subscriptions** in useEffect
3. **Create status visualization components** with appropriate styling
4. **Implement color coding functions** for new metrics
5. **Update test data generator** with realistic scenarios
6. **Add safety thresholds** and warning systems
7. **Update documentation** with new features

## License

This robot status dashboard is part of the ROS2 3D Point Cloud Visualization project and follows the same licensing terms.
