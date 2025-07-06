# Debug Services - Individual Terminal Commands

This guide helps you run each service individually in separate terminals to debug issues.

## Prerequisites

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd ~/ros2_ws

# Build workspace
colcon build --packages-select zed_rtabmap_demo

# Source workspace
source install/setup.bash
```

## Terminal 1: Zenoh RMW Daemon

```bash
# Terminal 1 - Zenoh RMW Daemon
source /opt/ros/humble/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

**Expected Output:**
- Should start without errors
- May show some initialization messages

---

## Terminal 2: ROSBridge WebSocket Server

```bash
# Terminal 2 - ROSBridge
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Expected Output:**
```
[INFO] [rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
```

**Test Connection:**
- Open browser: `http://localhost:9090`
- Should see "WebSocket connection established"

---

## Terminal 3: ZED 3D Mapping with RTAB-Map

```bash
# Terminal 3 - ZED 3D Mapping
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch zed_rtabmap_demo zed_3d_mapping.launch.py camera_model:=zed2i use_zed_odometry:=true launch_rviz:=false
```

**Expected Output:**
- ZED camera initialization messages
- RTAB-Map startup logs
- Point cloud and image topic advertisements

**Verify Topics:**
```bash
# In another terminal
ros2 topic list | grep zed
```

**Expected Topics:**
- `/zed/zed_node/rgb/image_rect_color`
- `/zed/zed_node/point_cloud/cloud_registered`
- `/zed/zed_node/odom`

---

## Terminal 4: Web Video Server

```bash
# Terminal 4 - Web Video Server
source /opt/ros/humble/setup.bash
ros2 run web_video_server web_video_server
```

**Expected Output:**
```
[INFO] [web_video_server]: Waiting for service /web_video_server/...
[INFO] [web_video_server]: Web server started on port 8080
```

**Test Camera Stream:**
- Open browser: `http://localhost:8080`
- Should see available camera topics
- Click on `/zed/zed_node/rgb/image_rect_color` to view stream

---

## Terminal 5: React Dashboard

```bash
# Terminal 5 - Dashboard
cd ~/ros2_ws/pointcloud-dashboard
npm start
```

**Expected Output:**
```
webpack compiled successfully
Local:            http://localhost:3002
```

**Test Dashboard:**
- Open browser: `http://localhost:3002`
- Navigate to Robot Control page
- Check browser console for connection logs

---

## Debugging Steps

### 1. Check Each Service Status

Run each terminal command above and verify:

1. **Zenoh RMW Daemon**: Should start without errors
2. **ROSBridge**: Should show "started on port 9090"
3. **ZED Mapping**: Should show camera initialization and topic advertisements
4. **Web Video Server**: Should show "started on port 8080"
5. **Dashboard**: Should compile and start on port 3002

### 2. Verify Topic Data Flow

```bash
# Check if topics exist
ros2 topic list

# Check if camera topic has data
ros2 topic echo /zed/zed_node/rgb/image_rect_color --once

# Check topic info
ros2 topic info /zed/zed_node/rgb/image_rect_color

# Check topic hz (frequency)
ros2 topic hz /zed/zed_node/rgb/image_rect_color
```

### 3. Test Web Video Server

```bash
# Check if web video server is serving camera
curl http://localhost:8080

# Check specific camera stream
curl http://localhost:8080/stream?topic=/zed/zed_node/rgb/image_rect_color
```

### 4. Test ROSBridge Connection

Open browser console on `http://localhost:3002/test_camera_connection.html` and check:

1. "Connected to websocket server" - ROSBridge working
2. Topic listing - Service availability
3. Camera subscription - Topic subscription
4. Image messages - Data flow

### 5. Common Issues and Solutions

#### Issue: ZED Camera Not Detected
```bash
# Check if ZED camera is connected
lsusb | grep -i zed

# Check ZED SDK installation
/usr/local/zed/tools/ZED_Explorer

# Test ZED camera directly
ros2 run zed_wrapper zed_wrapper
```

#### Issue: ROSBridge Not Connecting
```bash
# Check if port 9090 is in use
netstat -tulpn | grep 9090

# Test ROSBridge manually
ros2 run rosbridge_server rosbridge_websocket
```

#### Issue: Web Video Server Not Working
```bash
# Check if port 8080 is in use
netstat -tulpn | grep 8080

# Install web video server if missing
sudo apt install ros-humble-web-video-server

# Test with simple image topic
ros2 run image_publisher image_publisher
```

#### Issue: No Camera Data
```bash
# Check camera permissions
sudo chmod 666 /dev/video*

# Check if camera is being used by another process
sudo lsof /dev/video*

# Restart ZED daemon
sudo systemctl restart zed
```

### 6. Service Dependencies

Start services in this order:
1. Zenoh RMW Daemon (optional but recommended)
2. ZED 3D Mapping (provides camera topics)
3. ROSBridge (connects ROS to web)
4. Web Video Server (streams camera to web)
5. Dashboard (web interface)

### 7. Log Files

If running the setup script, check these log files:
- `/tmp/zenohd.log`
- `/tmp/rosbridge.log`
- `/tmp/zed_mapping.log`
- `/tmp/web_video_server.log`

### 8. Quick Test Commands

```bash
# Test all at once
ros2 topic list | grep -E "(zed|map|cmd_vel)"

# Test camera data
timeout 5 ros2 topic echo /zed/zed_node/rgb/image_rect_color --once

# Test web services
curl -s http://localhost:9090 && echo "ROSBridge OK"
curl -s http://localhost:8080 && echo "Web Video Server OK"
curl -s http://localhost:3002 && echo "Dashboard OK"
```

## Expected Final State

When all services are running correctly:

1. **5 terminals** running different services
2. **Camera feed** visible at `http://localhost:8080`
3. **Dashboard** working at `http://localhost:3002`
4. **Robot control** functional with joysticks
5. **Camera feed** showing in dashboard
6. **Occupancy map** displaying (if mapping data available)

This individual terminal approach will help identify exactly which service is causing issues and provide detailed error messages for debugging.
