# Robot Control Dashboard

A comprehensive web-based interface for remotely controlling your robot with real-time camera feed, joystick controls, and occupancy map visualization.

## Features

### 🎮 Robot Control Dashboard
- **Full-screen ZED camera feed** - Real-time video stream from `/zed/zed_node/rgb/image_rect_color`
- **Dual joystick controls** - Separate linear and angular movement controls
- **Interactive occupancy map** - 3D visualization of `/map` topic with expandable view
- **Real-time status monitoring** - Connection status, camera feed status, and velocity display
- **Emergency stop functionality** - Immediate robot halt capability

### 🗺️ Point Cloud Visualization (Original)
- Real-time 3D point cloud visualization from RTAB-Map
- Interactive 3D navigation and controls
- Point cloud statistics and information

## Navigation

The dashboard now includes two main pages:

1. **Point Cloud** (`/`) - Original 3D point cloud visualization
2. **Robot Control** (`/control`) - New robot control interface

Use the navigation bar at the top to switch between pages.

## Robot Control Interface Components

### Camera Feed Viewer
- **Topic**: `/zed/zed_node/rgb/image_rect_color`
- **Message Type**: `sensor_msgs/Image`
- **Features**:
  - Full-screen background video display
  - Support for RGB8, BGR8, and RGBA8 encodings
  - Live feed indicator
  - Automatic timeout detection

### Joystick Controller
- **Linear Movement Joystick**:
  - Forward/Backward movement (Y-axis)
  - Left/Right strafing (X-axis)
  - Configurable max linear speed (0.1-3.0 m/s)
- **Angular Movement Joystick**:
  - Rotation control (left/right)
  - Configurable max angular speed (0.1-3.0 rad/s)
- **Real-time velocity display**
- **Emergency stop button**

### Occupancy Map Viewer
- **Topic**: `/map`
- **Message Type**: `nav_msgs/OccupancyGrid`
- **Features**:
  - 3D grid visualization of occupied/free/unknown cells
  - Compact and expanded view modes
  - Interactive 3D navigation (in expanded mode)
  - Map statistics (size, resolution, cell counts)
  - Color-coded legend

### Status Overlay
- ROS connection status
- Camera feed status
- Active ROS topics information
- Real-time timestamps

## ROS Topics

### Published Topics
- `/cmd_vel` (geometry_msgs/Twist) - Robot velocity commands

### Subscribed Topics
- `/zed/zed_node/rgb/image_rect_color` (sensor_msgs/Image) - Camera feed
- `/map` (nav_msgs/OccupancyGrid) - Occupancy grid map

## Technical Implementation

### Architecture
- **React 17** with TypeScript
- **React Router 6** for navigation
- **Three.js** with React Three Fiber for 3D graphics
- **ROSBridge WebSocket** for ROS communication
- **CSS3** with modern styling and animations

### Key Components
```
src/components/
├── RobotControlDashboard.tsx    # Main control dashboard
├── CameraFeedViewer.tsx         # ZED camera feed display
├── JoystickController.tsx       # Dual joystick controls
├── OccupancyMapViewer.tsx       # 3D occupancy map
├── RobotStatusOverlay.tsx       # Status information
├── Navigation.tsx               # Navigation bar
└── PointCloudVisualization.tsx  # Original point cloud viewer
```

### Styling
- Responsive design for desktop and mobile
- Dark theme with cyan accents
- Overlay-based UI with backdrop blur effects
- Smooth animations and transitions

## Usage Instructions

### Prerequisites
1. **ROS2** environment with your robot
2. **ROSBridge Server** running on port 9090
3. **ZED Camera** publishing to the specified topic
4. **Navigation stack** publishing occupancy grid

### Starting the Dashboard
```bash
cd pointcloud-dashboard
npm install
npm start
```

The dashboard will be available at `http://localhost:8080`

### Robot Control
1. Navigate to the **Robot Control** page using the navigation bar
2. Ensure ROS connection is established (green indicator)
3. Wait for camera feed to activate
4. Use joysticks to control robot movement:
   - **Left joystick**: Linear movement (forward/back/strafe)
   - **Right joystick**: Angular movement (rotation)
5. Adjust speed limits using the sliders
6. Monitor occupancy map in bottom-left corner
7. Use emergency stop if needed

### Joystick Controls
- **Mouse/Touch**: Click and drag within joystick circles
- **Release**: Automatically returns to center and stops movement
- **Visual Feedback**: Knob color changes when active
- **Real-time Values**: Current joystick positions displayed

### Map Interaction
- **Compact Mode**: Small overlay with basic stats
- **Expanded Mode**: Click expand button for full 3D interaction
- **3D Navigation**: Mouse controls for pan, zoom, rotate (expanded mode)
- **Statistics**: Real-time cell counts and map information

## Configuration

### Speed Limits
- Linear speed: 0.1 - 3.0 m/s (configurable via sliders)
- Angular speed: 0.1 - 3.0 rad/s (configurable via sliders)

### ROS Bridge
- Default URL: `ws://localhost:9090`
- Configurable in the status overlay

### Camera Settings
- Automatic encoding detection (RGB8/BGR8/RGBA8)
- 5-second timeout for feed detection
- Canvas-based rendering for optimal performance

## Troubleshooting

### Common Issues

1. **No Camera Feed**
   - Check if ZED camera is publishing to the correct topic
   - Verify ROSBridge connection
   - Ensure topic name matches: `/zed/zed_node/rgb/image_rect_color`

2. **Robot Not Moving**
   - Verify `/cmd_vel` topic is being received by robot
   - Check joystick is active (colored knob)
   - Ensure emergency stop is not engaged

3. **No Occupancy Map**
   - Check if navigation stack is running
   - Verify `/map` topic is being published
   - Ensure map data is not empty

4. **Connection Issues**
   - Verify ROSBridge server is running: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
   - Check firewall settings
   - Ensure correct WebSocket URL

### Performance Optimization
- Use hardware acceleration for 3D rendering
- Limit point cloud density for better performance
- Close unused browser tabs
- Use modern browsers (Chrome, Firefox, Safari)

## Development

### Adding New Features
1. Create new component in `src/components/`
2. Add routing in `App.tsx` if needed
3. Update navigation in `Navigation.tsx`
4. Add corresponding CSS styles

### Customization
- Modify colors in CSS files
- Adjust joystick sensitivity in `JoystickController.tsx`
- Change topic names in respective components
- Update speed limits and ranges as needed

## Safety Considerations

- **Emergency Stop**: Always accessible via dedicated button
- **Speed Limits**: Configurable maximum speeds for safety
- **Visual Feedback**: Clear indicators for all robot states
- **Connection Monitoring**: Real-time status of all connections
- **Timeout Handling**: Automatic stop on connection loss

## Future Enhancements

- [ ] Gamepad/controller support
- [ ] Voice commands
- [ ] Waypoint navigation
- [ ] Multiple camera feeds
- [ ] Robot arm control
- [ ] Sensor data visualization
- [ ] Recording and playback
- [ ] Multi-robot support

## License

This project is part of the ROS2 Point Cloud Dashboard and follows the same licensing terms.
