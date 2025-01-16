import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from math import sin, cos, pi

class TFBroadcasterNode(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.broadcast_transforms)  # Update every 50 ms

    def broadcast_transforms(self):
        # Transform from odom to base_link
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

        # Transform from base_link to laser_frame
        t.header.frame_id = "base_link"
        t.child_frame_id = "laser_frame"
        t.transform.translation.x = 0.2  # Adjust based on actual lidar mounting position
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1
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
