import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # --- CONFIGURATION ---
    # Default to FALSE for live flying, TRUE if testing with bag
    use_sim_time = LaunchConfiguration('use_sim_time') 
    database_path = LaunchConfiguration('database_path')
    
    # 1. RTAB-Map Parameters (Localization Mode)
    parameters=[{
          'frame_id': 'base_link',
          'odom_frame_id': 'odom',
          'subscribe_depth': True,
          'subscribe_odom_info': False, # Usually False for localization to save bandwidth
          'approx_sync': True,
          'wait_imu_to_init': False,
          'use_sim_time': use_sim_time,
          'database_path': database_path,
          
          # --- KEY LOCALIZATION CHANGES ---
          'Mem/IncrementalMemory': 'false',  # STOP Mapping (Localization only)
          'Mem/InitWMWithAllNodes': 'true',  # Load the whole map into RAM
          
          # Performance Tuning (Optional for Pi)
          'Rtabmap/DetectionRate': '2',      # Run detection at 2Hz
          'Kp/MaxFeatures': '400',           # Limit features to save CPU

          # Sync Tolerance
          'queue_size': 50,
          'approx_sync_max_interval': 0.1,
          
          # QoS Compatibility
          'qos_image': 2,
          'qos_camera_info': 2,
    }]

    # 2. Topic Remappings (Matches your "Fixed" topics)
    remappings=[
          ('imu', '/mavros/imu/data'),
          ('rgb/image', '/camera/fixed'),            # Consumes data from restamper
          ('rgb/camera_info', '/camera_info/fixed'), 
          ('depth/image', '/depth_camera/fixed'),    
          ('odom', '/mavros/local_position/odom')
    ]

    return LaunchDescription([
        
        # Args
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Set true for replay'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db'),

        # --- BRIDGE 1: Odom Topic -> TF Transform ---
        # Ensures map->odom->base_link chain exists
        Node(
            package='midcone_rtabmap',
            executable='odom_to_tf',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('odom', '/mavros/local_position/odom')] 
        ),

        # --- BRIDGE 2: Base -> Camera Transform ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.1', '--y', '0', '--z', '0', 
                         '--yaw', '-1.5707', '--pitch', '0', '--roll', '-1.5707', 
                         '--frame-id', 'base_link', 
                         '--child-frame-id', 'camera_link']
        ),

        # --- BRIDGE 3: Fix Camera Timestamps (The Restamper) ---
        # Takes real camera topics and publishes to /camera/fixed
        Node(
            package='midcone_rtabmap',
            executable='restamper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # --- MAIN LOCALIZATION NODE ---
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            # NOTE: removed arguments=['-d'] so we DO NOT delete the map
        ),

        # --- VISUALIZER ---
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings
        ),
    ])