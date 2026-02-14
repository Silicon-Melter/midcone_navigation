from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # --- CONFIGURATION ---
    # We force Sim Time to TRUE because this is for the simulation workflow
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    
    # 1. RTAB-Map Parameters (Localization Mode)
    parameters=[{
          'frame_id': 'base_link',
          'odom_frame_id': 'odom',
          'subscribe_depth': True,
          'subscribe_odom_info': False, # We use our own odom
          'approx_sync': True,
          'wait_imu_to_init': False, # Don't wait for IMU if using external odom
          'use_sim_time': use_sim_time,
          'database_path': database_path,
          
          # LOCALIZATION MODE
          'Mem/IncrementalMemory': 'false', 
          'Mem/InitWMWithAllNodes': 'true', 
          
          'queue_size': 20,
          'qos_image': 2,
          'qos_camera_info': 2,
    }]

    # 2. Topic Remappings (Matches our "Clean Slate" topics)
    remappings=[
          ('imu', '/mavros/imu/data'),
          ('rgb/image', '/camera/camera/color/image_raw/uncompressed'), 
          ('rgb/camera_info', '/camera/camera/aligned_depth_to_color/camera_info'), 
          ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw/uncompressed'),
          ('odom', '/mavros/local_position/odom')
    ]

    return LaunchDescription([
        
        DeclareLaunchArgument('database_path', default_value='~/mavros.db'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # --- BRIDGE 1: Odom Topic -> TF Transform ---
        # Using our new "Foolproof" bridge from midcone_sim_v2
        Node(
            package='midcone_navigation',
            executable='odom_to_tf',
            output='screen',
            # FIX: Pass use_sim_time here so the node starts in the correct time mode
            parameters=[{'use_sim_time': use_sim_time}] 
        ),

        # --- BRIDGE 2: Base -> Camera Transform ---
        # FORCED SIM TIME to prevent "TF OLD DATA"
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments = ['0.1', '0', '0', '-1.57', '0', '-1.57', 'base_link', 'camera_link']
        ),

        # --- MAIN LOCALIZATION NODE ---
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
        ),

        # --- VISUALIZER ---
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings
        ),
    ])