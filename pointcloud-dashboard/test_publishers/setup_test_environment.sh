#!/bin/bash

# Setup Test Environment for ROS2 Dashboard Test Publishers
# This script helps set up the testing environment quickly

set -e  # Exit on any error

echo "🚀 ROS2 Dashboard Test Environment Setup"
echo "========================================"

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

# Check if ROS2 is sourced
check_ros2() {
    print_info "Checking ROS2 environment..."
    
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS2 not sourced. Please run:"
        echo "  source /opt/ros/humble/setup.bash"
        echo "  # or your ROS2 distribution"
        exit 1
    else
        print_status "ROS2 $ROS_DISTRO environment detected"
    fi
}

# Check Python dependencies
check_python_deps() {
    print_info "Checking Python dependencies..."
    
    local missing_deps=()
    
    # Check each required package
    if ! python3 -c "import rclpy" 2>/dev/null; then
        missing_deps+=("rclpy")
    fi
    
    if ! python3 -c "import numpy" 2>/dev/null; then
        missing_deps+=("numpy")
    fi
    
    if ! python3 -c "import cv2" 2>/dev/null; then
        missing_deps+=("opencv-python")
    fi
    
    if [ ${#missing_deps[@]} -eq 0 ]; then
        print_status "All Python dependencies found"
    else
        print_warning "Missing Python packages: ${missing_deps[*]}"
        echo "Install with: pip install ${missing_deps[*]}"
        
        read -p "Install missing packages now? (y/n): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            pip install "${missing_deps[@]}"
            print_status "Dependencies installed"
        else
            print_error "Please install missing dependencies manually"
            exit 1
        fi
    fi
}

# Check if rosbridge is installed
check_rosbridge() {
    print_info "Checking rosbridge installation..."
    
    if ros2 pkg list | grep -q rosbridge_server; then
        print_status "rosbridge_server found"
    else
        print_warning "rosbridge_server not found"
        echo "Install with: sudo apt install ros-$ROS_DISTRO-rosbridge-suite"
        
        read -p "Install rosbridge_suite now? (y/n): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            sudo apt update
            sudo apt install ros-$ROS_DISTRO-rosbridge-suite
            print_status "rosbridge_suite installed"
        else
            print_error "Please install rosbridge_suite manually"
            exit 1
        fi
    fi
}

# Make scripts executable
make_executable() {
    print_info "Making test publisher scripts executable..."
    
    # Get the directory where this script is located
    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    
    # Change to the script directory
    cd "$script_dir"
    
    # Check if Python files exist
    if ls *.py 1> /dev/null 2>&1; then
        chmod +x *.py
        print_status "Scripts are now executable"
    else
        print_warning "No Python files found in $script_dir"
        print_info "Available files:"
        ls -la
    fi
}

# Start rosbridge in background
start_rosbridge() {
    print_info "Checking if rosbridge is already running..."
    
    if pgrep -f "rosbridge" > /dev/null; then
        print_warning "rosbridge is already running"
        return 0
    fi
    
    print_info "Starting rosbridge_server..."
    
    # Start rosbridge in background
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge.log 2>&1 &
    ROSBRIDGE_PID=$!
    
    # Wait a moment for it to start
    sleep 3
    
    # Check if it's running
    if kill -0 $ROSBRIDGE_PID 2>/dev/null; then
        print_status "rosbridge_server started (PID: $ROSBRIDGE_PID)"
        echo $ROSBRIDGE_PID > /tmp/rosbridge.pid
        return 0
    else
        print_error "Failed to start rosbridge_server"
        cat /tmp/rosbridge.log
        return 1
    fi
}

# Test a single publisher
test_publisher() {
    local publisher_name=$1
    local script_name=$2
    
    print_info "Testing $publisher_name publisher..."
    
    # Start publisher in background
    python3 "$script_name" > "/tmp/${publisher_name}_test.log" 2>&1 &
    local pid=$!
    
    # Wait a moment
    sleep 2
    
    # Check if it's still running
    if kill -0 $pid 2>/dev/null; then
        print_status "$publisher_name publisher is working"
        kill $pid
        return 0
    else
        print_error "$publisher_name publisher failed"
        cat "/tmp/${publisher_name}_test.log"
        return 1
    fi
}

# Run basic tests
run_tests() {
    print_info "Running basic publisher tests..."
    
    local failed_tests=()
    
    # Test each publisher
    test_publisher "Battery" "battery_test_publisher.py" || failed_tests+=("battery")
    test_publisher "IMU" "imu_test_publisher.py" || failed_tests+=("imu")
    test_publisher "System" "system_monitoring_test_publisher.py" || failed_tests+=("system")
    test_publisher "Point Cloud" "pointcloud_test_publisher.py" || failed_tests+=("pointcloud")
    test_publisher "Map" "occupancy_map_test_publisher.py" || failed_tests+=("map")
    test_publisher "Camera" "camera_test_publisher.py" || failed_tests+=("camera")
    test_publisher "Robot Control" "robot_control_test_publisher.py" || failed_tests+=("robot")
    
    if [ ${#failed_tests[@]} -eq 0 ]; then
        print_status "All publisher tests passed!"
    else
        print_warning "Some tests failed: ${failed_tests[*]}"
        print_info "Check individual publisher logs in /tmp/ for details"
    fi
}

# Show usage instructions
show_usage() {
    echo
    print_info "🎯 Quick Start Commands:"
    echo
    echo "1. Run all test publishers:"
    echo "   python3 run_all_test_publishers.py"
    echo
    echo "2. Run specific publishers:"
    echo "   python3 run_all_test_publishers.py --publishers battery,imu,system"
    echo
    echo "3. List available publishers:"
    echo "   python3 run_all_test_publishers.py --list"
    echo
    echo "4. Start React dashboard (in another terminal):"
    echo "   cd ../  # Go to pointcloud-dashboard directory"
    echo "   npm start"
    echo
    echo "5. Open dashboard: http://localhost:3000"
    echo
    print_info "📚 For detailed documentation, see README.md"
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
    
    # Clean up log files
    rm -f /tmp/*_test.log /tmp/rosbridge.log
}

# Main execution
main() {
    # Parse command line arguments
    local run_tests_flag=false
    local start_rosbridge_flag=false
    local cleanup_flag=false
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --test)
                run_tests_flag=true
                shift
                ;;
            --start-rosbridge)
                start_rosbridge_flag=true
                shift
                ;;
            --cleanup)
                cleanup_flag=true
                shift
                ;;
            --help|-h)
                echo "Usage: $0 [OPTIONS]"
                echo
                echo "Options:"
                echo "  --test              Run basic publisher tests"
                echo "  --start-rosbridge   Start rosbridge_server"
                echo "  --cleanup           Clean up processes and temp files"
                echo "  --help, -h          Show this help message"
                echo
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                echo "Use --help for usage information"
                exit 1
                ;;
        esac
    done
    
    # Handle cleanup
    if [ "$cleanup_flag" = true ]; then
        cleanup
        exit 0
    fi
    
    # Run setup checks
    check_ros2
    check_python_deps
    check_rosbridge
    make_executable
    
    # Start rosbridge if requested
    if [ "$start_rosbridge_flag" = true ]; then
        start_rosbridge
    fi
    
    # Run tests if requested
    if [ "$run_tests_flag" = true ]; then
        run_tests
    fi
    
    # Show usage instructions
    show_usage
    
    print_status "Setup complete! 🎉"
}

# Set up signal handlers
trap cleanup EXIT INT TERM

# Run main function with all arguments
main "$@"
