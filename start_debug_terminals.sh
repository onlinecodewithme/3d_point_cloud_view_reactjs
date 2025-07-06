#!/bin/bash

# Debug Terminal Launcher
# This script helps open multiple terminals with the right commands for debugging

echo "🔍 Debug Terminal Launcher"
echo "========================="

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_header() {
    echo -e "${BLUE}$1${NC}"
}

# Check if we're in the right directory
if [ ! -d "pointcloud-dashboard" ]; then
    echo "❌ Error: pointcloud-dashboard directory not found!"
    echo "Please run this script from the ros2_ws directory."
    exit 1
fi

print_header "🚀 Starting Debug Terminals..."

# Function to open terminal with command
open_terminal() {
    local title="$1"
    local command="$2"
    local wait_msg="$3"
    
    print_info "Opening terminal: $title"
    
    # Try different terminal emulators
    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --title="$title" -- bash -c "$command; echo 'Press Enter to close...'; read"
    elif command -v xterm &> /dev/null; then
        xterm -title "$title" -e bash -c "$command; echo 'Press Enter to close...'; read" &
    elif command -v konsole &> /dev/null; then
        konsole --title "$title" -e bash -c "$command; echo 'Press Enter to close...'; read" &
    else
        print_warning "No terminal emulator found. Please run manually:"
        echo "  $command"
        return 1
    fi
    
    if [ ! -z "$wait_msg" ]; then
        print_info "$wait_msg"
        sleep 2
    fi
}

# Terminal 1: Zenoh RMW Daemon
open_terminal "1. Zenoh RMW Daemon" \
    "source /opt/ros/\$ROS_DISTRO/setup.bash && echo 'Starting Zenoh RMW Daemon...' && ros2 run rmw_zenoh_cpp rmw_zenohd" \
    "Waiting for Zenoh to start..."

# Terminal 2: ROSBridge
open_terminal "2. ROSBridge WebSocket" \
    "source /opt/ros/\$ROS_DISTRO/setup.bash && echo 'Starting ROSBridge WebSocket Server...' && ros2 launch rosbridge_server rosbridge_websocket_launch.xml" \
    "Waiting for ROSBridge to start..."

# Terminal 3: ZED 3D Mapping
open_terminal "3. ZED 3D Mapping" \
    "source /opt/ros/\$ROS_DISTRO/setup.bash && cd ~/ros2_ws && source install/setup.bash && echo 'Starting ZED 3D Mapping...' && ros2 launch zed_rtabmap_demo zed_3d_mapping.launch.py camera_model:=zed2i use_zed_odometry:=true launch_rviz:=false" \
    "Waiting for ZED mapping to start..."

# Terminal 4: Web Video Server
open_terminal "4. Web Video Server" \
    "source /opt/ros/\$ROS_DISTRO/setup.bash && echo 'Starting Web Video Server...' && ros2 run web_video_server web_video_server" \
    "Waiting for Web Video Server to start..."

# Terminal 5: Dashboard
open_terminal "5. React Dashboard" \
    "cd ~/ros2_ws/pointcloud-dashboard && echo 'Starting React Dashboard...' && npm start" \
    "Waiting for Dashboard to start..."

print_header "✅ All terminals launched!"
echo ""
print_info "Debug terminals are now opening. Check each terminal for:"
echo "  1. Zenoh RMW Daemon - Should start without errors"
echo "  2. ROSBridge - Should show 'started on port 9090'"
echo "  3. ZED Mapping - Should show camera initialization"
echo "  4. Web Video Server - Should show 'started on port 8080'"
echo "  5. Dashboard - Should show 'webpack compiled successfully'"
echo ""
print_info "Test URLs:"
echo "  • Dashboard: http://localhost:3002"
echo "  • Camera Test: http://localhost:3002/test_camera_connection.html"
echo "  • Web Video Server: http://localhost:8080"
echo ""
print_info "For detailed debugging steps, see: DEBUG_SERVICES.md"
echo ""
print_warning "If terminals don't open automatically, run commands manually from DEBUG_SERVICES.md"
