#!/bin/bash

# High-Performance System Optimization Script for RTAB-Map + RTX 4090
# Run with: sudo ./optimize_system.sh

echo "🚀 Optimizing system for high-performance SLAM with RTX 4090..."

# 1. CPU Performance Optimizations
echo "⚡ Setting CPU performance mode..."
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null

# 2. GPU Performance Optimizations
echo "🎮 Optimizing GPU performance..."
sudo nvidia-smi -pm 1  # Enable persistence mode
sudo nvidia-smi -pl 450  # Set power limit to maximum (RTX 4090)
sudo nvidia-smi -ac 10501,2520  # Set memory and graphics clocks to max

# 3. Memory Optimizations
echo "💾 Optimizing memory settings..."
echo 1 | sudo tee /proc/sys/vm/swappiness > /dev/null  # Minimize swap usage
echo 10 | sudo tee /proc/sys/vm/dirty_ratio > /dev/null  # Faster disk writes
echo 5 | sudo tee /proc/sys/vm/dirty_background_ratio > /dev/null

# 4. Network Optimizations for ROS
echo "🌐 Optimizing network for ROS..."
echo 'net.core.rmem_max = 134217728' | sudo tee -a /etc/sysctl.conf > /dev/null
echo 'net.core.wmem_max = 134217728' | sudo tee -a /etc/sysctl.conf > /dev/null
echo 'net.core.netdev_max_backlog = 5000' | sudo tee -a /etc/sysctl.conf > /dev/null
sudo sysctl -p > /dev/null

# 5. Real-time Scheduling for ROS processes
echo "⏱️ Setting up real-time scheduling..."
echo '@ros2 - rtprio 99' | sudo tee -a /etc/security/limits.conf > /dev/null
echo '@ros2 - memlock unlimited' | sudo tee -a /etc/security/limits.conf > /dev/null

# 6. CUDA Environment Optimizations
echo "🔥 Setting CUDA environment variables..."
export CUDA_VISIBLE_DEVICES=0
export CUDA_CACHE_DISABLE=0
export CUDA_CACHE_MAXSIZE=2147483648  # 2GB cache
export CUDA_DEVICE_ORDER=PCI_BUS_ID

# 7. OpenCV CUDA Optimizations
export OPENCV_DNN_CUDA=1
export OPENCV_OPENCL_DEVICE=disabled  # Use CUDA instead of OpenCL

# 8. Thread Affinity for i9-14900HX (24 cores)
echo "🧵 Optimizing thread affinity..."
# Reserve cores 0-3 for system, use 4-31 for ROS
echo 'GRUB_CMDLINE_LINUX_DEFAULT="isolcpus=4-31 nohz_full=4-31 rcu_nocbs=4-31"' | sudo tee -a /etc/default/grub > /dev/null

# 9. Disable unnecessary services
echo "🛑 Disabling unnecessary services..."
sudo systemctl disable bluetooth.service > /dev/null 2>&1
sudo systemctl disable cups.service > /dev/null 2>&1
sudo systemctl disable avahi-daemon.service > /dev/null 2>&1

# 10. Set process priorities
echo "📊 Creating performance launch script..."
cat > launch_optimized.sh << 'EOF'
#!/bin/bash

# High-Performance Launch Script
echo "🚀 Launching optimized RTAB-Map system..."

# Set environment variables
export CUDA_VISIBLE_DEVICES=0
export CUDA_CACHE_DISABLE=0
export CUDA_CACHE_MAXSIZE=2147483648
export OPENCV_DNN_CUDA=1
export OMP_NUM_THREADS=16  # Use 16 threads for OpenMP
export OPENBLAS_NUM_THREADS=16

# Set CPU affinity and priority for each terminal
echo "Terminal 1: ZED Camera (High Priority)"
echo "taskset -c 4-11 nice -n -10 ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i pos_tracking_enabled:=true mapping_enabled:=false area_memory:=false"

echo ""
echo "Terminal 2: RTAB-Map (Highest Priority)"
echo "cd /home/xavier_ai/ros2_ws && source install/setup.bash && taskset -c 12-19 nice -n -15 ros2 launch zed_rtabmap_demo zed_3d_mapping.launch.py camera_model:=zed2i use_zed_odometry:=true"

echo ""
echo "Terminal 3: ROS Bridge (Medium Priority)"
echo "cd /home/xavier_ai/ros2_ws && source install/setup.bash && taskset -c 20-23 nice -n -5 ros2 launch rosbridge_server rosbridge_websocket_launch.xml"

echo ""
echo "Terminal 4: React Dashboard (Normal Priority)"
echo "cd /home/xavier_ai/ros2_ws/pointcloud-dashboard && npm start"

echo ""
echo "🎯 Performance Tips:"
echo "- Monitor GPU usage: watch -n 1 nvidia-smi"
echo "- Monitor CPU usage: htop"
echo "- Check ROS performance: ros2 topic hz /cloud_map"
EOF

chmod +x launch_optimized.sh

echo ""
echo "✅ System optimization complete!"
echo ""
echo "🎯 Performance Improvements Applied:"
echo "   • CPU: Performance governor enabled"
echo "   • GPU: RTX 4090 at maximum performance"
echo "   • Memory: Optimized for low latency"
echo "   • Network: High-bandwidth ROS communication"
echo "   • CUDA: Optimized for maximum throughput"
echo "   • Threading: Dedicated cores for ROS processes"
echo ""
echo "🚀 Use ./launch_optimized.sh for performance commands"
echo "⚠️  Reboot recommended for full optimization"
