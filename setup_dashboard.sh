#!/bin/bash

# Point Cloud Dashboard Setup Script
# This script sets up and runs the real-time point cloud visualization dashboard

set -e  # Exit on any error

echo "🚀 Point Cloud Dashboard Setup Script"
echo "====================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}$1${NC}"
}

# Check if ROS2 is installed
check_ros2() {
    print_header "🔍 Checking ROS2 Installation..."
    
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS2 not found. Please install ROS2 and source the setup script."
        print_status "Example: source /opt/ros/humble/setup.bash"
        exit 1
    fi
    
    print_status "ROS2 $ROS_DISTRO detected ✓"
}

# Check if Node.js is installed
check_nodejs() {
    print_header "🔍 Checking Node.js Installation..."
    
    if ! command -v node &> /dev/null; then
        print_error "Node.js not found. Please install Node.js (v14 or newer)."
        print_status "Visit: https://nodejs.org/"
        exit 1
    fi
    
    NODE_VERSION=$(node --version)
    print_status "Node.js $NODE_VERSION detected ✓"
}

# Install ROS2 dependencies
install_ros_deps() {
    print_header "📦 Installing ROS2 Dependencies..."
    
    print_status "Installing rosbridge_suite..."
    sudo apt update
    sudo apt install -y ros-$ROS_DISTRO-rosbridge-suite
    
    print_status "Installing RTAB-Map dependencies..."
    sudo apt install -y ros-$ROS_DISTRO-rtabmap-ros
    
    print_status "Installing ZED SDK dependencies..."
    sudo apt install -y ros-$ROS_DISTRO-zed-wrapper || print_warning "ZED wrapper not available in apt, please install manually"
    
    print_status "Installing web video server for camera streaming..."
    sudo apt install -y ros-$ROS_DISTRO-web-video-server || print_warning "Web video server not available in apt"
    
    print_status "ROS2 dependencies installed ✓"
}

# Build ROS2 workspace
build_workspace() {
    print_header "🔨 Building ROS2 Workspace..."
    
    print_status "Building workspace..."
    source /opt/ros/$ROS_DISTRO/setup.bash
    
    # Build the workspace
    colcon build --packages-select zed_rtabmap_demo
    
    if [ $? -eq 0 ]; then
        print_status "Workspace built successfully ✓"
    else
        print_error "Failed to build workspace"
        exit 1
    fi
    
    # Source the workspace
    source install/setup.bash
    print_status "Workspace sourced ✓"
}

# Setup dashboard
setup_dashboard() {
    print_header "🛠️  Setting up Dashboard..."
    
    if [ ! -d "pointcloud-dashboard" ]; then
        print_error "pointcloud-dashboard directory not found!"
        print_status "Please run this script from the ros2_ws directory."
        exit 1
    fi
    
    cd pointcloud-dashboard
    
    print_status "Installing npm dependencies..."
    npm install
    
    print_status "Dashboard setup complete ✓"
    cd ..
}

# Check CUDA availability
check_cuda() {
    print_header "🎮 Checking CUDA Support..."
    
    if command -v nvidia-smi &> /dev/null; then
        print_status "NVIDIA GPU detected:"
        nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader,nounits
        
        print_status "Testing CUDA accelerator..."
        cd pointcloud-dashboard
        python3 cuda_accelerator.py
        cd ..
    else
        print_warning "NVIDIA GPU not detected. Dashboard will use CPU processing."
    fi
}

# Start services
start_services() {
    print_header "🚀 Starting Services..."
    
    print_status "Starting Zenoh RMW daemon..."
    print_warning "This will start zenohd and rosbridge in the background."
    print_warning "Use 'pkill -f zenohd' and 'pkill -f rosbridge' to stop them later."
    
    # Start Zenoh RMW daemon in background
    source /opt/ros/$ROS_DISTRO/setup.bash
    ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenohd.log 2>&1 &
    ZENOHD_PID=$!
    
    # Wait a moment for zenohd to start
    sleep 2
    
    # Check if zenohd is running
    if ps -p $ZENOHD_PID > /dev/null; then
        print_status "Zenoh RMW daemon started (PID: $ZENOHD_PID) ✓"
    else
        print_error "Failed to start Zenoh RMW daemon. Check /tmp/zenohd.log for details."
        print_warning "Continuing without Zenoh daemon..."
    fi
    
    print_status "Starting ROS bridge..."
    
    # Start rosbridge in background
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge.log 2>&1 &
    ROSBRIDGE_PID=$!
    
    # Wait a moment for rosbridge to start
    sleep 3
    
    # Check if rosbridge is running
    if ps -p $ROSBRIDGE_PID > /dev/null; then
        print_status "ROS bridge started (PID: $ROSBRIDGE_PID) ✓"
    else
        print_error "Failed to start ROS bridge. Check /tmp/rosbridge.log for details."
        exit 1
    fi
    
    # Build workspace again before launching ZED mapping
    print_status "Building workspace before launching ZED mapping..."
    source /opt/ros/$ROS_DISTRO/setup.bash
    colcon build --packages-select zed_rtabmap_demo
    
    if [ $? -eq 0 ]; then
        print_status "Workspace rebuilt successfully ✓"
    else
        print_error "Failed to rebuild workspace"
        print_warning "Continuing with existing build..."
    fi
    
    # Start ZED 3D mapping
    print_status "Starting ZED 3D mapping with RTAB-Map..."
    source install/setup.bash
    
    ros2 launch zed_rtabmap_demo zed_3d_mapping.launch.py camera_model:=zed2i use_zed_odometry:=true launch_rviz:=false > /tmp/zed_mapping.log 2>&1 &
    ZED_MAPPING_PID=$!
    
    # Wait a moment for ZED mapping to start
    sleep 5
    
    # Check if ZED mapping is running
    if ps -p $ZED_MAPPING_PID > /dev/null; then
        print_status "ZED 3D mapping started (PID: $ZED_MAPPING_PID) ✓"
    else
        print_error "Failed to start ZED 3D mapping. Check /tmp/zed_mapping.log for details."
        print_warning "Continuing without ZED mapping..."
        print_status "You can start it manually with:"
        print_status "  ros2 launch zed_rtabmap_demo zed_3d_mapping.launch.py camera_model:=zed2i use_zed_odometry:=true launch_rviz:=false"
    fi
    
    # Start web video server for camera streaming
    print_status "Starting web video server for camera streaming..."
    ros2 run web_video_server web_video_server > /tmp/web_video_server.log 2>&1 &
    WEB_VIDEO_PID=$!
    
    # Wait a moment for web video server to start
    sleep 2
    
    # Check if web video server is running
    if ps -p $WEB_VIDEO_PID > /dev/null; then
        print_status "Web video server started (PID: $WEB_VIDEO_PID) ✓"
        print_status "Camera streams available at: http://localhost:8080"
    else
        print_warning "Failed to start web video server. Check /tmp/web_video_server.log for details."
        print_status "Camera feed will use direct ROSBridge method"
    fi
    
    # Check for ZED camera topics
    print_status "Checking for ZED camera topics..."
    sleep 2
    
    # Use a more robust method to check for ZED topics
    ZED_TOPICS=$(ros2 topic list 2>/dev/null | grep "zed" || true)
    if [ ! -z "$ZED_TOPICS" ]; then
        print_status "ZED camera topics detected ✓"
        echo "$ZED_TOPICS" | while read topic; do
            print_status "  Found: $topic"
        done
        
        # Check specific topics for data
        print_status "Checking topic data flow..."
        
        # Check camera image topic
        if echo "$ZED_TOPICS" | grep -q "image_rect_color"; then
            IMAGE_TOPIC=$(echo "$ZED_TOPICS" | grep "image_rect_color" | head -1)
            print_status "Testing image topic: $IMAGE_TOPIC"
            timeout 3 ros2 topic echo "$IMAGE_TOPIC" --once > /dev/null 2>&1 && \
                print_status "  ✓ Image data flowing" || \
                print_warning "  ⚠ No image data detected"
        fi
        
        # Check point cloud topic
        if echo "$ZED_TOPICS" | grep -q "point_cloud"; then
            PC_TOPIC=$(echo "$ZED_TOPICS" | grep "point_cloud" | head -1)
            print_status "Testing point cloud topic: $PC_TOPIC"
            timeout 3 ros2 topic echo "$PC_TOPIC" --once > /dev/null 2>&1 && \
                print_status "  ✓ Point cloud data flowing" || \
                print_warning "  ⚠ No point cloud data detected"
        fi
        
        # Check for map topic
        print_status "Checking for map topic..."
        if ros2 topic list 2>/dev/null | grep -q "/map"; then
            print_status "Testing map topic: /map"
            timeout 3 ros2 topic echo "/map" --once > /dev/null 2>&1 && \
                print_status "  ✓ Map data flowing" || \
                print_warning "  ⚠ No map data detected"
        else
            print_warning "  ⚠ /map topic not found"
        fi
        
    else
        print_warning "ZED camera topics not detected. Dashboard will run in demo mode."
    fi
    
    print_status "Starting dashboard..."
    cd pointcloud-dashboard
    
    print_status "Dashboard will be available at: http://localhost:3002"
    print_status "Press Ctrl+C to stop all services."
    
    # Trap Ctrl+C to cleanup
    trap cleanup INT
    
    # Start dashboard (this will block)
    npm start
}

# Cleanup function
cleanup() {
    print_header "🧹 Cleaning up..."
    
    if [ ! -z "$ZENOHD_PID" ]; then
        print_status "Stopping Zenoh RMW daemon (PID: $ZENOHD_PID)..."
        kill $ZENOHD_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$ROSBRIDGE_PID" ]; then
        print_status "Stopping ROS bridge (PID: $ROSBRIDGE_PID)..."
        kill $ROSBRIDGE_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$ZED_MAPPING_PID" ]; then
        print_status "Stopping ZED 3D mapping (PID: $ZED_MAPPING_PID)..."
        kill $ZED_MAPPING_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$WEB_VIDEO_PID" ]; then
        print_status "Stopping web video server (PID: $WEB_VIDEO_PID)..."
        kill $WEB_VIDEO_PID 2>/dev/null || true
    fi
    
    # Kill any remaining processes
    pkill -f zenohd 2>/dev/null || true
    pkill -f rosbridge 2>/dev/null || true
    pkill -f "zed_3d_mapping" 2>/dev/null || true
    pkill -f "rtabmap" 2>/dev/null || true
    pkill -f "web_video_server" 2>/dev/null || true
    
    print_status "Cleanup complete. Goodbye! 👋"
    exit 0
}

# Main execution
main() {
    print_header "Starting Point Cloud Dashboard Setup..."
    
    # Parse command line arguments
    SKIP_INSTALL=false
    START_ONLY=false
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --skip-install)
                SKIP_INSTALL=true
                shift
                ;;
            --start-only)
                START_ONLY=true
                shift
                ;;
            --help|-h)
                echo "Usage: $0 [OPTIONS]"
                echo ""
                echo "Options:"
                echo "  --skip-install    Skip installation steps"
                echo "  --start-only      Only start services (skip all setup)"
                echo "  --help, -h        Show this help message"
                echo ""
                echo "Examples:"
                echo "  $0                    # Full setup and start"
                echo "  $0 --skip-install    # Skip installation, setup dashboard and start"
                echo "  $0 --start-only      # Only start services"
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                print_status "Use --help for usage information"
                exit 1
                ;;
        esac
    done
    
    if [ "$START_ONLY" = true ]; then
        print_status "Starting services only..."
        check_ros2
        start_services
        return
    fi
    
    # Run setup steps
    check_ros2
    check_nodejs
    
    if [ "$SKIP_INSTALL" = false ]; then
        install_ros_deps
    fi
    
    build_workspace
    setup_dashboard
    check_cuda
    start_services
}

# Run main function with all arguments
main "$@"
