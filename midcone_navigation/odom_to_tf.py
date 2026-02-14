import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SimpleBridge(Node):
    def __init__(self):
        super().__init__('simple_bridge')
        
        # REMOVED: self.set_parameters(...) 
        # relying on the Launch file ensures the Clock is initialized correctly.
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # 2. PUBLISH STATIC CAMERA TF (Immediate on startup)
        # This replaces the static_transform_publisher node
        self.publish_camera_tf()

        # 3. SETUP SUBSCRIBER (Best Effort for MAVROS)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            Odometry, '/mavros/local_position/odom', self.handle_odom, qos_profile)
            
        # Log which clock we are using to be sure
        if self.get_parameter_or('use_sim_time', rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, False)).value:
            self.get_logger().info("✅ BRIDGE MODE: SIMULATION TIME (Correct)")
        else:
            self.get_logger().warn("⚠️ BRIDGE MODE: SYSTEM TIME (Check Launch File!)")

        self.get_logger().info("✅ CONSOLIDATED BRIDGE STARTED (Odom + Camera TF)")

    def publish_camera_tf(self):
        t = TransformStamped()
        # This will now correctly return Sim Time (e.g. 33s) instead of Real Time
        t.header.stamp = self.get_clock().now().to_msg() 
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        
        # Camera Position (0.1m forward)
        t.transform.translation.x = 0.1
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Camera Rotation (Optical Frame: Roll -90, Yaw -90)
        t.transform.rotation.x = -0.5
        t.transform.rotation.y = 0.5
        t.transform.rotation.z = -0.5
        t.transform.rotation.w = 0.5
        
        self.static_broadcaster.sendTransform(t)

    def handle_odom(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = SimpleBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()