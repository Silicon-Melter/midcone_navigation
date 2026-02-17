import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # --- ARGUMENTS ---
    # In real-time, we typically use the system clock, so default is False
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    database_path = DeclareLaunchArgument('database_path', default_value='~/mavros.db')

    # --- RTAB-MAP CONFIGURATION ---
    parameters=[{
          'frame_id': 'base_link',
          'odom_frame_id': 'odom', # MAVROS provides this frame
          
          # Real-Time Settings
          'subscribe_depth': True,
          'subscribe_odom_info': False, # We use Mavros Odom, not RTAB-Map's
          'approx_sync': True,          # Still needed for RealSense RGB-Depth sync
          'wait_imu_to_init': False,
          'use_sim_time': LaunchConfiguration('use_sim_time'),
          'database_path': LaunchConfiguration('database_path'),
          
          # LOCALIZATION MODE (Robot will not map, only find itself)
          # Change to 'true' if you want to create a new map
          'Mem/IncrementalMemory': 'false', 
          'Mem/InitWMWithAllNodes': 'true', 
          
          # Optimization
          'queue_size': 20,
          'qos_image': 2,
          'qos_camera_info': 2,
    }]

    # --- TOPIC REMAPPINGS ---
    # These match the 'realsense_node' config in your bringup file
    remappings=[
          ('imu', '/mavros/imu/data'),
          ('odom', '/mavros/local_position/odom'),

          # Raw Topics from RealSense (No Decompressor needed)
          ('rgb/image', '/camera/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/camera/color/camera_info'),
          ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw')
    ]

    return LaunchDescription([
        use_sim_time,
        database_path,

        # --- BRIDGE: Odom -> TF ---
        # Converts MAVROS Odometry message into a TF for RTAB-Map
        Node(
            package='midcone_navigation', # Replace with your package name
            executable='odom_to_tf',
            name='odom_to_tf_bridge',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # --- OPTIONAL: STATIC TRANSFORM ---
        # Only uncomment this if your URDF does NOT connect 'base_link' to 'camera_link'
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_base_link_broadcaster',
        #     # Adjust X Y Z Yaw Pitch Roll as needed
        #     arguments=['0.1', '0', '0', '0', '0', '0', 'base_link', 'camera_link'], 
        # ),

        # --- MAIN NODE: RTAB-MAP ---
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            # No '-d' argument -> Loads the database defined in parameters
        ),

        # --- OPTIONAL: VISUALIZER ---
        # Node(
        #    package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #    parameters=parameters,
        #    remappings=remappings
        # ),
    ])