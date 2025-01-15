import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from math import sin, cos, pi

class TFBroadcasterNode(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_node')

        # Inițializăm transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer pentru actualizare periodică
        self.timer = self.create_timer(0.05, self.broadcast_transforms)

    def broadcast_transforms(self):
        # Transformare de la `odom` la `base_link`
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # Transformare de la `base_link` la `laser_frame`
        t.header.frame_id = "base_link"
        t.child_frame_id = "laser_frame"
        t.transform.translation.x = 0.2  # Ajustează poziția Lidar-ului
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # Transformare de la `base_link` la `imu_frame`
        t.header.frame_id = "base_link"
        t.child_frame_id = "imu_frame"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.05  # Ajustează poziția IMU-ului
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterNode()
    rclpy.spin(node)
    rclpy.shutdown()
