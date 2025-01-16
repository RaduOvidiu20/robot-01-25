import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

class FakeOdometryNode(Node):
    def __init__(self):
        super().__init__('fake_odometry_node')
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.publish_odometry)  # Publish every 100ms
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Simulate motion
        linear_speed = 0.1  # m/s
        angular_speed = 0.1  # rad/s

        dt = 0.1
        self.theta += angular_speed * dt
        self.x += linear_speed * math.cos(self.theta) * dt
        self.y += linear_speed * math.sin(self.theta) * dt

        # Populate odometry message
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(self.theta / 2.0),
            w=math.cos(self.theta / 2.0),
        )

        odom.twist.twist.linear.x = linear_speed
        odom.twist.twist.angular.z = angular_speed

        self.publisher_.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = FakeOdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()
