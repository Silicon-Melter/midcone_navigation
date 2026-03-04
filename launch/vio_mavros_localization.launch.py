import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # --- 1. ARGUMENTS ---
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    
    # Using os.path.expanduser safely resolves '~' to '/home/user/'
    database_path = DeclareLaunchArgument(
        'database_path', 
        default_value=os.path.expanduser('~/mavros.db')
    )

    # --- 2. VIO CONFIGURATION (rgbd_odometry) ---
    vio_parameters = [{
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'wait_imu_to_init': True,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        
        # RTAB-Map specific parameters MUST be strings
        'Odom/Strategy': '0',
        'Odom/GuessMotion': 'True',
        'Odom/ResetCountdown': '1',
        'Vis/FeatureType': '8',
        'publish_tf': False,  # EKF will handle TF
    }]

    vio_remappings = [
        ('imu', '/mavros/imu/data'),
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
        ('odom', '/visual_odom') # Internal topic sent to EKF
    ]

    # --- 3. EKF FUSION CONFIGURATION (robot_localization) ---
    ekf_config = {
        'frequency': 30.0,
        'sensor_timeout': 0.1,
        'two_d_mode': False,
        'publish_tf': True, # Publishes odom -> base_link
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_link_frame': 'base_link',
        'world_frame': 'odom',
        'print_diagnostics': True,

        # INPUT 1: VIO (Trust Position & Linear Velocity)
        'odom0': '/visual_odom',
        'odom0_config': [True,  True,  True,   # X, Y, Z
                         False, False, False,  # Roll, Pitch, Yaw (Ignored)
                         True,  True,  True,   # Vx, Vy, Vz
                         False, False, False,  # VRoll, VPitch, VYaw
                         False, False, False], # Ax, Ay, Az
        'odom0_differential': False,
        'odom0_relative': False,

        # INPUT 2: IMU (Trust Orientation & Angular Velocity)
        'imu0': '/mavros/imu/data',
        'imu0_config': [False, False, False, 
                        True,  True,  True,    # Roll, Pitch, Yaw
                        False, False, False, 
                        True,  True,  True,    # VRoll, VPitch, VYaw
                        True,  True,  True],   # Ax, Ay, Az
        'imu0_differential': False,
        'imu0_relative': False,
        'imu0_remove_gravitational_acceleration': True,
    }

    # --- 4. MAP LOCALIZATION CONFIGURATION (rtabmap) ---
    slam_parameters = [{
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_odom_info': False, 
        'approx_sync': True,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'database_path': LaunchConfiguration('database_path'),
        
        # STRICT LOCALIZATION OVERRIDES (Must be strings)
        'Mem/IncrementalMemory': 'False', 
        'Mem/InitWMWithAllNodes': 'True',
        'RGBD/NeighborLinkRefining': 'True',
        
        'queue_size': 20,
    }]

    slam_remappings = [
        ('imu', '/mavros/imu/data'),
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
        ('odom', '/mavros/odometry/out') # Listen to the fused EKF output
    ]

    return LaunchDescription([
        use_sim_time,
        database_path,

        # NODE 1: Visual Odometry (VIO)
        Node(
            package='rtabmap_odom', 
            executable='rgbd_odometry', 
            output='screen',
            parameters=vio_parameters,
            remappings=vio_remappings
        ),

        # NODE 2: EKF Fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_fusion',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('odometry/filtered', '/mavros/odometry/out') # Direct to PX4
            ]
        ),

        # NODE 3: Map Localization (SLAM Engine)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=slam_parameters,
            remappings=slam_remappings,
            arguments=[] # Empty arguments ensures NO '-d' (does not delete map)
        ),

        # NODE 4: Visualizer (rtabmap_viz)
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=slam_parameters, # Shares the exact same parameters as SLAM
            remappings=slam_remappings
        ),

        # NODE 5: Auto-Download Map Trigger
        # Waits 5 seconds for the database to load, then forces rtabmap_viz to show the cloud
        TimerAction(
            period=5.0, 
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', 
                        '/rtabmap/publish_map', 
                        'rtabmap_msgs/srv/PublishMap', 
                        '{global_map: true, optimized: true, graph_only: false}'
                    ],
                    output='screen'
                )
            ]
        ),
    ])