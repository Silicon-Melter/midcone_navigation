import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    database_path = DeclareLaunchArgument('database_path', default_value='~/mavros.db')

    # ---------------------------------------------------------
    # 1. EKF CONFIGURATION (robot_localization)
    # ---------------------------------------------------------
    ekf_config = {
        'frequency': 30.0,
        'sensor_timeout': 0.1,
        'two_d_mode': False,          # FALSE because drones fly in 3D
        'publish_tf': True,           # Publish odom -> base_link
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_link_frame': 'base_link',
        'world_frame': 'odom',
        'print_diagnostics': True,    # CRITICAL: Tells you why it fails in /diagnostics

        # INPUT 1: VIO (Trust Position & Velocity)
        'odom0': '/visual_odom',
        # X, Y, Z, Roll, Pitch, Yaw, Vx, Vy, Vz, Vroll, Vpitch, Vyaw, Ax, Ay, Az
        'odom0_config': [True,  True,  True,   # Trust X, Y, Z
                         False, False, False,  # Ignore VIO orientation (use IMU)
                         True,  True,  True,   # Trust linear velocity
                         False, False, False, 
                         False, False, False],
        'odom0_differential': False,
        'odom0_relative': False,

        # INPUT 2: IMU (Trust Orientation & Angular Velocity)
        'imu0': '/mavros/imu/data',
        'imu0_config': [False, False, False, 
                        True,  True,  True,    # Trust Roll, Pitch, Yaw
                        False, False, False, 
                        True,  True,  True,    # Trust Angular Velocity
                        True,  True,  True],   # Trust Linear Acceleration
        'imu0_differential': False,
        'imu0_relative': False,
        'imu0_remove_gravitational_acceleration': True, # MAVROS IMU includes gravity
    }

    # ---------------------------------------------------------
    # 2. VIO CONFIGURATION
    # ---------------------------------------------------------
    vio_parameters = [{
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'publish_tf': False,  # EKF handles TF now
    }]

    # ---------------------------------------------------------
    # 3. LOCALIZATION (MAP) CONFIGURATION
    # ---------------------------------------------------------
    slam_parameters = [{
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_odom_info': False, # Changed: Listen to standard odom msg
        'approx_sync': True,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'database_path': LaunchConfiguration('database_path'),
        'Mem/IncrementalMemory': 'false', 
        'Mem/InitWMWithAllNodes': 'true',
    }]

    return LaunchDescription([
        use_sim_time,
        database_path,

        # 1. VIO Node
        Node(
            package='rtabmap_odom', 
            executable='rgbd_odometry', 
            output='screen',
            parameters=vio_parameters,
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                ('odom', '/visual_odom') # Send to EKF
            ]
        ),

        # 2. EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_fusion',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                # Output directly to PX4
                ('odometry/filtered', '/mavros/odometry/out') 
            ]
        ),

        # 3. Map Localization Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=slam_parameters,
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                ('odom', '/mavros/odometry/out') # Listen to EKF
            ]
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'subscribe_depth': True,
                'subscribe_odom_info': False, # Listening to the EKF odom
                'approx_sync': True,
                'queue_size': 20,
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                ('odom', '/mavros/odometry/out') # Listen to the fused EKF output
            ]
        ),
    ])