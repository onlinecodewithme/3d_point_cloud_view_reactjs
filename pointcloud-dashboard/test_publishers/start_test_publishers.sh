#!/bin/bash

# Start Test Publishers with ROS2 Environment
# This script automatically sources ROS2 and starts the test publishers

set -e  # Exit on any error

echo "🚀 ROS2 Dashboard Test Publishers Launcher"
echo "============================================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

# Function to find and source ROS2
source_ros2() {
    print_info "Looking for ROS2 installation..."
    
    # Common ROS2 installation paths
    local ros2_paths=(
        "/opt/ros/humble/setup.bash"
        "/opt/ros/iron/setup.bash"
        "/opt/ros/jazzy/setup.bash"
        "/opt/ros/galactic/setup.bash"
        "/opt/ros/foxy/setup.bash"
        "$HOME/ros2_install/setup.bash"
        "$HOME/ros2_ws/install/setup.bash"
    )
    
    # Check if ROS2 is already sourced
    if [ ! -z "$ROS_DISTRO" ]; then
        print_status "ROS2 $ROS_DISTRO already sourced"
        return 0
    fi
    
    # Try to find and source ROS2
    for ros_path in "${ros2_paths[@]}"; do
        if [ -f "$ros_path" ]; then
            print_info "Found ROS2 setup at: $ros_path"
            source "$ros_path"
            
            if [ ! -z "$ROS_DISTRO" ]; then
                print_status "Successfully sourced ROS2 $ROS_DISTRO"
                return 0
            fi
        fi
    done
    
    # If not found, try to detect from common locations
    if [ -d "/opt/ros" ]; then
        print_info "Checking /opt/ros directory..."
        local latest_distro=$(ls /opt/ros | sort -r | head -n1)
        if [ ! -z "$latest_distro" ] && [ -f "/opt/ros/$latest_distro/setup.bash" ]; then
            print_info "Found ROS2 $latest_distro, attempting to source..."
            source "/opt/ros/$latest_distro/setup.bash"
            
            if [ ! -z "$ROS_DISTRO" ]; then
                print_status "Successfully sourced ROS2 $ROS_DISTRO"
                return 0
            fi
        fi
    fi
    
    print_error "Could not find or source ROS2 installation"
    print_info "Please install ROS2 or manually source it before running this script:"
    echo "  source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS2 distro"
    return 1
}

# Function to start rosbridge if needed
start_rosbridge() {
    print_info "Checking rosbridge status..."
    
    if pgrep -f "rosbridge" > /dev/null; then
        print_status "rosbridge is already running"
        return 0
    fi
    
    print_info "Starting rosbridge_server..."
    
    # Check if rosbridge is installed
    if ! ros2 pkg list | grep -q rosbridge_server; then
        print_error "rosbridge_server not found"
        print_info "Install with: sudo apt install ros-$ROS_DISTRO-rosbridge-suite"
        return 1
    fi
    
    # Start rosbridge in background
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge.log 2>&1 &
    local rosbridge_pid=$!
    
    # Wait for it to start
    sleep 3
    
    # Check if it's running
    if kill -0 $rosbridge_pid 2>/dev/null; then
        print_status "rosbridge_server started (PID: $rosbridge_pid)"
        echo $rosbridge_pid > /tmp/rosbridge.pid
        return 0
    else
        print_error "Failed to start rosbridge_server"
        print_info "Check log: /tmp/rosbridge.log"
        return 1
    fi
}

# Function to run test publishers
run_publishers() {
    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    cd "$script_dir"
    
    print_info "Starting test publishers..."
    
    # Check if the Python launcher exists
    if [ ! -f "run_all_test_publishers.py" ]; then
        print_error "run_all_test_publishers.py not found in $script_dir"
        return 1
    fi
    
    # Export ROS environment variables to ensure they're passed to subprocess
    export ROS_DISTRO
    export ROS_VERSION
    export ROS_PYTHON_VERSION
    export AMENT_PREFIX_PATH
    export CMAKE_PREFIX_PATH
    export LD_LIBRARY_PATH
    export PATH
    export PKG_CONFIG_PATH
    export PYTHONPATH
    export ROS_LOCALHOST_ONLY
    export ROS_DOMAIN_ID
    
    print_info "ROS Environment exported: ROS_DISTRO=$ROS_DISTRO"
    
    # Run the Python launcher with all arguments passed to this script
    python3 run_all_test_publishers.py "$@"
}

# Cleanup function
cleanup() {
    print_info "Cleaning up..."
    
    # Stop rosbridge if we started it
    if [ -f /tmp/rosbridge.pid ]; then
        local pid=$(cat /tmp/rosbridge.pid)
        if kill -0 $pid 2>/dev/null; then
            kill $pid
            print_status "Stopped rosbridge_server"
        fi
        rm -f /tmp/rosbridge.pid
    fi
}

# Show help
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo
    echo "This script automatically sources ROS2 and starts the test publishers."
    echo
    echo "Options:"
    echo "  --help, -h              Show this help message"
    echo "  --no-rosbridge          Don't start rosbridge automatically"
    echo "  --publishers LIST       Comma-separated list of publishers to run"
    echo "  --no-battery           Skip battery test publisher"
    echo "  --no-imu               Skip IMU test publisher"
    echo "  --no-system            Skip system monitoring test publisher"
    echo "  --no-pointcloud        Skip point cloud test publisher"
    echo "  --no-map               Skip occupancy map test publisher"
    echo "  --no-camera            Skip camera test publisher"
    echo "  --no-robot             Skip robot control test publisher"
    echo "  --list                 List available publishers and exit"
    echo
    echo "Examples:"
    echo "  $0                                    # Run all publishers"
    echo "  $0 --list                            # List available publishers"
    echo "  $0 --publishers battery,imu,system   # Run specific publishers"
    echo "  $0 --no-camera --no-pointcloud       # Run all except camera and pointcloud"
    echo
    echo "Dashboard URLs:"
    echo "  Main Dashboard: http://localhost:3000"
    echo "  Battery Monitor: http://localhost:3000/battery"
    echo "  System Monitor: http://localhost:3000/system-monitoring"
    echo "  IMU Sensor: http://localhost:3000/imu-sensor"
    echo "  Robot Control: http://localhost:3000/control"
}

# Main function
main() {
    # Parse arguments for help
    for arg in "$@"; do
        case $arg in
            --help|-h)
                show_help
                exit 0
                ;;
        esac
    done
    
    # Check for --no-rosbridge flag
    local start_rosbridge_flag=true
    local filtered_args=()
    
    for arg in "$@"; do
        case $arg in
            --no-rosbridge)
                start_rosbridge_flag=false
                ;;
            *)
                filtered_args+=("$arg")
                ;;
        esac
    done
    
    # Source ROS2
    if ! source_ros2; then
        exit 1
    fi
    
    # Start rosbridge if requested
    if [ "$start_rosbridge_flag" = true ]; then
        if ! start_rosbridge; then
            print_warning "Failed to start rosbridge, but continuing..."
            print_info "You may need to start rosbridge manually:"
            echo "  ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
        fi
    fi
    
    # Run the test publishers
    run_publishers "${filtered_args[@]}"
}

# Set up signal handlers
trap cleanup EXIT INT TERM

# Run main function
main "$@"
