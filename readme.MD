export PATH=/usr/local/cuda-12.6/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash

# Cognition Brain ROS2 Network Configuration
source $HOME/.cognition_brain_ros2_config
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/targets/aarch64-linux/lib:$LD_LIBRARY_PATH
export PATH=/usr/local/cuda-12.6/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda-12.6
export PATH=/usr/local/cuda-12.6/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda-12.6

# CUDA Optimizations for AI
export CUDA_CACHE_DISABLE=0
export CUDA_CACHE_MAXSIZE=2147483648
export CUDA_CACHE_PATH=/tmp/cuda_cache
export CUDA_DEVICE_ORDER=PCI_BUS_ID
export CUDA_VISIBLE_DEVICES=0

# Ollama Optimizations
export OLLAMA_NUM_PARALLEL=1
export OLLAMA_MAX_LOADED_MODELS=1
export OLLAMA_FLASH_ATTENTION=1
export OLLAMA_GPU_OVERHEAD=0.1
export OLLAMA_KEEP_ALIVE=5m

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
#export ROS_LOCALHOST_ONLY=0
. "$HOME/.cargo/env"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI=~/autonomous_robot_cognition_brain/config/routerconfig.json5

#export LD_LIBRARY_PATH=/opt/ros/humble/opt/zenoh_cpp_vendor/lib:$LD_LIBRARY_PATH
#export ZENOH_SESSION_CONFIG_URI=$HOME/.zenoh/session_config.json5



ros2 run rmw_zenoh_cpp rmw_zenohd
ros2 run demo_nodes_cpp listner
