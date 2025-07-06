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
