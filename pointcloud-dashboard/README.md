# Point Cloud Dashboard

A real-time point cloud visualization dashboard for ROS2, built with React, Three.js, and TypeScript. This application connects to ROS2 via rosbridge and visualizes point cloud data from the ZED camera in an interactive 3D environment.

## Features

- **Real-time Visualization**: Live point cloud data streaming from ROS2
- **Interactive 3D View**: Rotate, pan, and zoom with mouse controls
- **Color-coded Points**: Height-based coloring or RGB data from the sensor
- **Connection Management**: WebSocket connection to ROS bridge with auto-reconnect
- **Performance Monitoring**: Real-time statistics and point count display
- **Demo Mode**: Generates sample data when not connected to ROS

## Prerequisites

1. **ROS2** (Humble, Iron, or newer)
2. **rosbridge_suite** for WebSocket communication
3. **Node.js** (v14 or newer)
4. **ZED Camera** with ROS2 wrapper (optional, demo mode available)
5. **NVIDIA GPU** with CUDA support (optional, for acceleration)

## Installation

### 1. Install ROS Bridge

```bash
sudo apt install ros-$ROS_DISTRO-rosbridge-suite
```

### 2. Install Dependencies

```bash
cd pointcloud-dashboard
npm install
```

### 3. Optional: CUDA Acceleration Setup

For enhanced performance with NVIDIA GPUs:

```bash
# Check CUDA availability
nvidia-smi

# Install Python CUDA dependencies (optional)
pip install cupy-cuda12x  # For CUDA 12.x
pip install numba

# Test CUDA accelerator
python3 cuda_accelerator.py
```

The CUDA accelerator automatically falls back to CPU processing if CUDA is unavailable.

## Quick Start Guide

### Complete Setup (All-in-One)

```bash
# 1. Install ROS2 dependencies
sudo apt install ros-$ROS_DISTRO-rosbridge-suite

# 2. Setup dashboard
cd pointcloud-dashboard
npm install

# 3. Start ROS bridge (Terminal 1)
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# 4. Start ZED camera (Terminal 2, if available)
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 &

# 5. Start dashboard (Terminal 3)
npm start

# Dashboard will be available at: http://localhost:3002
```

### Verify Setup

```bash
# Check ROS bridge is running
ros2 node list | grep rosbridge

# Check point cloud topic (if ZED camera connected)
ros2 topic list | grep point_cloud
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered

# Test CUDA acceleration (optional)
python3 cuda_accelerator.py
```

## Usage

### 1. Start ROS Bridge

In a terminal, start the ROS bridge server:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

This will start the WebSocket server on `ws://localhost:9090`.

### 2. Start ZED Camera (Optional)

If you have a ZED camera, start the ZED node:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

### 3. Start the Dashboard

```bash
npm start
```

The dashboard will open in your browser at `http://localhost:3002`.

## Configuration

### ROS Bridge URL

The default WebSocket URL is `ws://localhost:9090`. You can change this in the connection panel of the dashboard.

### Point Cloud Topic

The dashboard subscribes to `/zed/zed_node/point_cloud/cloud_registered` by default. To change this, modify the topic name in `src/components/RosConnection.tsx`.

## Controls

- **Left Click + Drag**: Rotate the view
- **Right Click + Drag**: Pan the view
- **Mouse Wheel**: Zoom in/out
- **Connection Panel**: Connect/disconnect from ROS bridge

## Dashboard Features

### Connection Status
- Real-time connection status indicator
- Automatic reconnection on connection loss
- Manual connect/disconnect controls

### Point Cloud Information
- Live point count display
- Last update timestamp
- Topic information
- Frame rate monitoring

### Visualization
- 3D point cloud rendering with Three.js
- Color-coded points (height-based or RGB)
- Grid and axes helpers for reference
- Performance statistics overlay

## Demo Mode

When not connected to ROS, the dashboard automatically generates demo point cloud data to showcase the visualization capabilities. This includes:
- Animated sphere pattern
- Color-coded points
- Real-time updates

## Troubleshooting

### Connection Issues

1. **Check ROS Bridge**: Ensure rosbridge is running on the correct port
   ```bash
   ros2 node list | grep rosbridge
   ```

2. **Firewall**: Make sure port 9090 is not blocked

3. **CORS Issues**: rosbridge_suite handles CORS automatically

### Performance Issues

1. **Point Count**: Large point clouds (>100K points) may impact performance
2. **Browser**: Use Chrome or Firefox for best WebGL performance
3. **Hardware**: Ensure your GPU supports WebGL 2.0

### ZED Camera Issues

1. **Topic Check**: Verify the point cloud topic is publishing
   ```bash
   ros2 topic list | grep point_cloud
   ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
   ```

2. **Permissions**: Ensure proper USB permissions for ZED camera

## Development

### Build for Production

```bash
npm run build
```

### Development Mode

```bash
npm run dev
```

This opens the dashboard with hot reload enabled.

### Project Structure

```
pointcloud-dashboard/
├── src/
│   ├── components/
│   │   ├── Dashboard.tsx          # Status and info panel
│   │   ├── PointCloudVisualization.tsx  # 3D rendering
│   │   └── RosConnection.tsx      # WebSocket connection
│   ├── App.tsx                    # Main application
│   ├── App.css                    # Styles
│   └── index.tsx                  # Entry point
├── public/
│   └── index.html                 # HTML template
├── webpack.config.js              # Build configuration
├── tsconfig.json                  # TypeScript configuration
└── package.json                   # Dependencies
```

## Technical Details

### Point Cloud Parsing

The application parses ROS2 `sensor_msgs/PointCloud2` messages:
- Extracts XYZ coordinates from binary data
- Handles RGB color information when available
- Converts ROS timestamps to JavaScript timestamps
- Supports both big-endian and little-endian data

### WebSocket Protocol

Uses rosbridge protocol for ROS communication:
- Subscribe to topics with JSON messages
- Real-time data streaming
- Automatic message parsing and validation

### 3D Rendering

Built with Three.js and React Three Fiber:
- Efficient point cloud rendering with BufferGeometry
- Vertex colors for point coloring
- Interactive camera controls
- Performance monitoring

## License

ISC License

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test with real ROS2 data
5. Submit a pull request

## Support

For issues and questions:
1. Check the troubleshooting section
2. Verify ROS2 and rosbridge setup
3. Test with demo mode first
4. Check browser console for errors
