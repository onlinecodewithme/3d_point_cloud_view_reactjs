# Requirements:
#   A ZED camera
#   Install zed ros2 wrapper package (https://github.com/stereolabs/zed-ros2-wrapper)
# Example:
#   $ ros2 launch zed_rtabmap_demo zed_3d_mapping.launch.py camera_model:=zed2i

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition, IfCondition
from tf2_ros import StaticTransformBroadcaster

import tempfile

def launch_setup(context: LaunchContext, *args, **kwargs):

    # Hack to override grab_resolution parameter without changing any files
    with tempfile.NamedTemporaryFile(mode='w+t', delete=False) as zed_override_file:
        zed_override_file.write("---\n"+
                  "/**:\n"+
                  "    ros__parameters:\n"+
                  "        general:\n"+
                  "            grab_resolution: 'VGA'\n"+
                  "        depth:\n"+
                  "            quality: 'PERFORMANCE'\n"+
                  "        point_cloud:\n"+
                  "            map_resolution: 0.05")

    # Common parameters for all RTAB-Map nodes
    parameters=[{
        'frame_id': 'zed_camera_link',
        'subscribe_rgbd': True,
        'subscribe_imu': True,  # Enable IMU for accurate navigation
        'approx_sync': True,  # Use approximate sync for IMU
        'wait_imu_to_init': True,  # Wait for IMU for accurate initialization
        
        # 3D Grid mapping parameters for stable high-density mapping
        'Grid/3D': 'true',
        'Grid/RangeMax': '12.0',  # Stable range
        'Grid/CellSize': '0.03',  # 3cm cell size for good detail
        'Grid/DepthDecimation': '1',  # Keep all depth points
        'Grid/RayTracing': 'true',
        'Grid/3DMap': 'true',
        'Grid/GroundIsObstacle': 'false',
        'Grid/MaxObstacleHeight': '3.0',  # Standard height
        'Grid/MaxGroundHeight': '0.1',  # Standard ground height
        'Grid/ClusterRadius': '0.1',  # Standard clustering
        'Grid/MinClusterSize': '10',  # Standard cluster size
        'Grid/FlatObstacleDetected': 'true',
        'Grid/PreVoxelFiltering': 'true',  # Re-enable for stability
        'Grid/PostFiltering': 'true',  # Re-enable for stability
        
        # RTAB-Map's parameters for pure odometry mode (no loop closure at all)
        'Rtabmap/DetectionRate': '0.0',  # Disable loop closure detection completely
        'Rtabmap/CreateIntermediateNodes': 'false',  # Disable to prevent resets
        'Rtabmap/LinearUpdate': '0.1',  # Standard update rate
        'Rtabmap/AngularUpdate': '0.1',  # Standard update rate
        'Rtabmap/TimeThr': '0',
        'Rtabmap/MaxRetrieved': '0',  # Don't retrieve any old nodes
        'Rtabmap/StartNewMapOnLoopClosure': 'false',  # Don't start new maps
        'Rtabmap/StatisticsLogsBufferedSize': '0',  # Disable statistics buffering
        
        # Memory management for continuous mapping (no loop closure)
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        'Mem/RecentWmRatio': '0.2',  # Limit recent working memory
        
        # Registration
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'false',
        
        # ICP parameters for point cloud registration
        'Icp/VoxelSize': '0.02',  # Reduced for higher density
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/MaxTranslation': '2.0',
        'Icp/MaxRotation': '1.0',
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '30',
        'Icp/Epsilon': '0.001',
        'Icp/Strategy': '1',
        'Icp/OutlierRatio': '0.7',
        'Icp/CorrespondenceRatio': '0.01',
        
        # Cloud parameters for stable high-density mapping
        'cloud_decimation': '1',  # Keep all points (no decimation)
        'cloud_max_depth': '12.0',  # Stable maximum depth
        'cloud_min_depth': '0.3',   # Standard minimum depth
        'cloud_voxel_size': '0.01',  # 1cm voxel size for good detail
        'cloud_noise_filtering_radius': '0.05',  # Standard noise filtering
        'cloud_noise_filtering_min_neighbors': '5',  # Standard neighbors
        'cloud_floor_culling_height': '0.0',
        'cloud_ceiling_culling_height': '3.0',  # Standard ceiling height
        
        # Visual features for stable loop closure (RGB-D only, no laser)
        'Vis/MaxFeatures': '1000',
        'Vis/MinInliers': '20',  # Increased for stricter loop closure
        'Vis/InlierDistance': '0.03',  # Reduced for stricter matching
        'Vis/EstimationType': '1',
        
        # Disable laser scan requirements (we only have RGB-D camera)
        'RGBD/ProximityBySpace': 'false',  # Disable laser-based proximity
        'RGBD/ProximityByTime': 'true',   # Use time-based proximity instead
        'RGBD/ProximityMaxGraphDepth': '0',  # Disable graph-based proximity
        'RGBD/ProximityPathMaxNeighbors': '0',  # Disable path-based proximity
        
        # Completely disable loop closure for continuous mapping
        'Rtabmap/LoopThr': '0.0',  # Disable loop closure completely
        'Mem/STMSize': '1',  # Minimal memory to prevent loop detection
        'Mem/BadSignaturesIgnored': 'true',  # Ignore bad signatures
        'RGBD/OptimizeFromGraphEnd': 'false',  # Disable optimization
        'RGBD/OptimizeMaxError': '0.0',  # No optimization
        'Rtabmap/LoopRatio': '0.0',  # No loop closure ratio
        'Mem/RehearsalSimilarity': '0.0',  # Disable rehearsal
        'Rtabmap/MaxRetrieved': '0',  # Don't retrieve old nodes
        'Mem/UseOdomFeatures': 'false',  # Don't use odometry features for loop detection
        'Mem/NotLinkedNodesKept': 'false',  # Don't keep unlinked nodes
        'Mem/ReduceGraph': 'false',  # Don't reduce graph
        'Kp/NNStrategy': '0',  # Disable nearest neighbor search
        
        # Keypoint detector
        'Kp/MaxFeatures': '1000',
        'Kp/DetectorStrategy': '6',
        'SURF/HessianThreshold': '200',
        
        # Optimization
        'Optimizer/Strategy': '1',
        'Optimizer/Iterations': '20',
        'Optimizer/Epsilon': '0.00001',
        'Optimizer/Robust': 'false',
    }]

    # Common remappings with IMU for accurate navigation
    remappings=[('imu', '/zed/zed_node/imu/data')]

    # Use ZED odometry if specified, otherwise use RTAB-Map's visual odometry
    if LaunchConfiguration('use_zed_odometry').perform(context) in ["True", "true"]:
        remappings.append(('odom', '/zed/zed_node/odom'))
    else:
        parameters[0]['subscribe_odom_info'] = True
    
    nodes = [
        # Static transforms for robot navigation
        # Base link to camera link transform (adjust based on your robot setup)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'zed_camera_link'],
            output='screen'
        ),
        
        # Camera link to IMU link transform (ZED2i specific)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_imu_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'zed_camera_link', 'zed_imu_link'],
            output='screen'
        ),
        
        # Map to odom transform (for navigation stack)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        
        # Note: ZED camera should already be running
        # We'll connect to the existing ZED topics
        
        # Sync rgb/depth/camera_info together
        Node(   
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=[('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
                        ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
                        ('depth/image', '/zed/zed_node/depth/depth_registered')]),

        # Visual odometry (only if not using ZED odometry)
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_zed_odometry')),
            parameters=parameters,
            remappings=remappings),

        # RTAB-Map SLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),  # Delete database on start
    ]

    # No GUI visualization - using React dashboard instead
    # This saves system resources and prevents GUI conflicts

    return nodes


def generate_launch_description():
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_zed_odometry', default_value='false',
            description='Use ZED\'s computed odometry instead of RTAB-Map\'s visual odometry.'),
        
        DeclareLaunchArgument(
            'camera_model', default_value='zed2i',
            description="The model of the camera. Valid choices: ['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm']"),
        
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Launch RViz2 instead of rtabmap_viz for visualization.'),
        
        OpaqueFunction(function=launch_setup)
    ])
