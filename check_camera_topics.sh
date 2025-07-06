#!/bin/bash

echo "🔍 Camera Topic Diagnostic Script"
echo "================================="

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_info() {
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

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS2 not sourced. Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

print_info "ROS2 $ROS_DISTRO detected"

# Source workspace if available
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    print_info "Workspace sourced"
fi

print_header "📋 Checking ROS2 Topics..."

# List all topics
print_info "All available topics:"
ros2 topic list

print_header "🔍 Looking for ZED camera topics..."

# Check for ZED topics specifically
ZED_TOPICS=$(ros2 topic list | grep -i zed || true)
if [ ! -z "$ZED_TOPICS" ]; then
    print_info "Found ZED topics:"
    echo "$ZED_TOPICS"
else
    print_warning "No ZED topics found"
fi

print_header "📹 Checking specific camera topic..."

# Check the specific camera topic
CAMERA_TOPIC="/zed/zed_node/rgb/image_rect_color"
if ros2 topic list | grep -q "$CAMERA_TOPIC"; then
    print_info "✅ Camera topic exists: $CAMERA_TOPIC"
    
    # Get topic info
    print_info "Topic info:"
    ros2 topic info "$CAMERA_TOPIC"
    
    # Check if topic has data
    print_info "Testing topic data (5 second timeout)..."
    if timeout 5 ros2 topic echo "$CAMERA_TOPIC" --once > /dev/null 2>&1; then
        print_info "✅ Topic has data!"
    else
        print_warning "⚠️ Topic exists but no data received in 5 seconds"
    fi
    
    # Check topic frequency
    print_info "Checking topic frequency (5 second test)..."
    timeout 5 ros2 topic hz "$CAMERA_TOPIC" || print_warning "Could not measure topic frequency"
    
else
    print_error "❌ Camera topic not found: $CAMERA_TOPIC"
fi

print_header "🌐 Checking ROSBridge..."

# Check if rosbridge is running
if pgrep -f rosbridge > /dev/null; then
    print_info "✅ ROSBridge process is running"
    
    # Check if port 9090 is open
    if netstat -tulpn 2>/dev/null | grep -q ":9090"; then
        print_info "✅ ROSBridge port 9090 is open"
    else
        print_warning "⚠️ Port 9090 not found in netstat"
    fi
else
    print_error "❌ ROSBridge process not running"
fi

print_header "🤖 Checking ZED node status..."

# Check if ZED node is running
if pgrep -f zed > /dev/null; then
    print_info "✅ ZED process is running"
else
    print_warning "⚠️ ZED process not found"
fi

# Check for rtabmap process
if pgrep -f rtabmap > /dev/null; then
    print_info "✅ RTAB-Map process is running"
else
    print_warning "⚠️ RTAB-Map process not found"
fi

print_header "📊 Summary"
echo "1. Check the console output above for any errors"
echo "2. If camera topic exists but has no data, restart ZED mapping"
echo "3. If ROSBridge is not running, start it with the debug script"
echo "4. Check browser console for detailed debugging info"
echo ""
print_info "To start services individually, use: ./start_debug_terminals.sh"
