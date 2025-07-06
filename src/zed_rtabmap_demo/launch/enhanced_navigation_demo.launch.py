#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    nav2 = LaunchConfiguration('nav2')
    rviz = LaunchConfiguration('rviz')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Whether to run SLAM'
    )
    
    declare_nav2_cmd = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Whether to run Nav2'
    )
    
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to run RViz'
    )

    # Get package directories
    zed_rtabmap_demo_dir = get_package_share_directory('zed_rtabmap_demo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # ZED Camera Node
    zed_node = Node(
        package='zed_wrapper',
        executable='zed_wrapper',
        name='zed_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'general.camera_model': 'zed2i',
            'general.camera_name': 'zed',
            'depth.depth_mode': 'PERFORMANCE',
            'depth.depth_stabilization': True,
            'point_cloud.point_cloud_freq': 10.0,
            'mapping.mapping_enabled': True,
            'mapping.resolution': 0.05,
            'mapping.max_mapping_range': 15.0,
            'pos_tracking.pos_tracking_enabled': True,
            'pos_tracking.publish_tf': True,
            'pos_tracking.publish_map_tf': True,
            'pos_tracking.map_frame': 'map',
            'pos_tracking.odometry_frame': 'odom',
            'pos_tracking.base_frame': 'base_link',
            'sensors.publish_imu_tf': True,
        }],
        remappings=[
            ('/zed/zed_node/point_cloud/cloud_registered', '/cloud_input'),
            ('/zed/zed_node/rgb/image_rect_color', '/camera/image'),
            ('/zed/zed_node/depth/depth_registered', '/camera/depth/image'),
        ]
    )

    # RTAB-Map SLAM Node
    rtabmap_slam = Node(
        condition=IfCondition(slam),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'wait_imu_to_init': True,
            
            # RTAB-Map parameters for enhanced navigation
            'Rtabmap/DetectionRate': '1.0',
            'Rtabmap/TimeThr': '0',
            'Rtabmap/MemoryThr': '0',
            'Rtabmap/CreateIntermediateNodes': 'true',
            'Rtabmap/StartNewMapOnLoopClosure': 'false',
            
            # Grid parameters for occupancy map
            'Grid/Sensor': '1',
            'Grid/3D': 'false',
            'Grid/RayTracing': 'true',
            'Grid/RangeMax': '15.0',
            'Grid/CellSize': '0.05',
            'Grid/ClusterRadius': '0.1',
            'Grid/GroundIsObstacle': 'false',
            'Grid/MaxGroundAngle': '45',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/MinClusterSize': '10',
            'Grid/NoiseFilteringRadius': '0.1',
            'Grid/NoiseFilteringMinNeighbors': '2',
            'Grid/NormalsSegmentation': 'false',
            'Grid/FlatObstacleDetected': 'true',
            
            # Visual odometry parameters
            'Vis/EstimationType': '1',
            'Vis/FeatureType': '6',
            'Vis/MaxDepth': '15.0',
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
            'Vis/RefineIterations': '5',
            
            # Loop closure parameters
            'Kp/DetectorStrategy': '6',
            'Kp/MaxFeatures': '400',
            'Kp/MaxDepth': '15.0',
            'Kp/SSC': 'true',
            
            # Memory management
            'Mem/RehearsalSimilarity': '0.6',
            'Mem/ImagePreDecimation': '2',
            'Mem/ImagePostDecimation': '1',
            'Mem/CompressionParallelized': 'true',
            
            # Optimization parameters
            'Optimizer/Strategy': '1',
            'Optimizer/Iterations': '20',
            'Optimizer/Epsilon': '0.00001',
            'Optimizer/Robust': 'true',
            'Optimizer/VarianceIgnored': 'false',
            
            # Database parameters
            'DbSqlite3/InMemory': 'false',
            'DbSqlite3/CacheSize': '10000',
            'DbSqlite3/JournalMode': '3',
            'DbSqlite3/Synchronous': '0',
            'DbSqlite3/TempStore': '2',
        }],
        remappings=[
            ('/rgb/image', '/camera/image'),
            ('/depth/image', '/camera/depth/image'),
            ('/scan_cloud', '/cloud_input'),
            ('/rtabmap/cloud_map', '/cloud_map'),
        ],
        arguments=['--delete_db_on_start']
    )

    # Map Server for saved maps
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': '',  # Will be set dynamically
        }]
    )

    # AMCL for localization when using saved maps
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'base_frame_id': 'base_link',
            'beam_skip_distance': 0.5,
            'beam_skip_error_threshold': 0.9,
            'beam_skip_threshold': 0.3,
            'do_beamskip': False,
            'global_frame_id': 'map',
            'lambda_short': 0.1,
            'laser_likelihood_max_dist': 2.0,
            'laser_max_range': 100.0,
            'laser_min_range': -1.0,
            'laser_model_type': 'likelihood_field',
            'max_beams': 60,
            'max_particles': 2000,
            'min_particles': 500,
            'odom_frame_id': 'odom',
            'pf_err': 0.05,
            'pf_z': 0.99,
            'recovery_alpha_fast': 0.0,
            'recovery_alpha_slow': 0.0,
            'resample_interval': 1,
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'save_pose_rate': 0.5,
            'sigma_hit': 0.2,
            'tf_broadcast': True,
            'transform_tolerance': 1.0,
            'update_min_a': 0.2,
            'update_min_d': 0.25,
            'z_hit': 0.5,
            'z_max': 0.05,
            'z_rand': 0.5,
            'z_short': 0.05,
            'scan_topic': '/scan'
        }]
    )

    # Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': PathJoinSubstitution([
                FindPackageShare('zed_rtabmap_demo'),
                'config',
                'nav2_params.yaml'
            ]),
            'use_lifecycle_mgr': 'false',
            'map_subscribe_transient_local': 'true'
        }.items(),
        condition=IfCondition(nav2)
    )

    # ROS Bridge Server for web dashboard
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0',
            'retry_startup_delay': 5.0,
            'fragment_timeout': 600,
            'delay_between_messages': 0,
            'max_message_size': None,
            'unregister_timeout': 10.0,
            'use_compression': False,
        }]
    )

    # Web Video Server for camera streams
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{
            'port': 8080,
            'server_threads': 1,
            'ros_threads': 2,
            'default_stream_type': 'mjpeg',
            'publish_rate': 30.0,
        }]
    )

    # Static transforms
    static_transforms = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_zed_camera_center',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'zed_camera_center']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'laser']
        )
    ]

    # RViz
    rviz_node = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('zed_rtabmap_demo'),
            'rviz',
            'enhanced_navigation.rviz'
        ])],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_nav2_cmd)
    ld.add_action(declare_rviz_cmd)

    # Add nodes
    ld.add_action(zed_node)
    ld.add_action(rtabmap_slam)
    ld.add_action(map_server)
    ld.add_action(amcl)
    ld.add_action(nav2_bringup)
    ld.add_action(rosbridge_server)
    ld.add_action(web_video_server)
    ld.add_action(rviz_node)

    # Add static transforms
    for transform in static_transforms:
        ld.add_action(transform)

    return ld
