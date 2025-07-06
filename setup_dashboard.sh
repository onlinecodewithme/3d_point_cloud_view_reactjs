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
    
    print_status "ROS2 dependencies installed ✓"
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
    
    print_status "Starting ROS bridge..."
    print_warning "This will start rosbridge in the background."
    print_warning "Use 'pkill -f rosbridge' to stop it later."
    
    # Start rosbridge in background
    source /opt/ros/$ROS_DISTRO/setup.bash
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
    
    # Check for ZED camera
    print_status "Checking for ZED camera..."
    if ros2 topic list | grep -q "zed"; then
        print_status "ZED camera topics detected ✓"
    else
        print_warning "ZED camera not detected. Dashboard will run in demo mode."
        print_status "To start ZED camera manually:"
        print_status "  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2"
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
    
    if [ ! -z "$ROSBRIDGE_PID" ]; then
        print_status "Stopping ROS bridge (PID: $ROSBRIDGE_PID)..."
        kill $ROSBRIDGE_PID 2>/dev/null || true
    fi
    
    # Kill any remaining rosbridge processes
    pkill -f rosbridge 2>/dev/null || true
    
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
    
    setup_dashboard
    check_cuda
    start_services
}

# Run main function with all arguments
main "$@"
