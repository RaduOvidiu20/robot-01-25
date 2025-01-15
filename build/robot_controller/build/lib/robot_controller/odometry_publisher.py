import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import sin, cos, pi

class OdometryPublisherNode(Node):
    def __init__(self):
        super().__init__('odometry_publisher_node')
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_time = self.get_clock().now()

        self.create_subscription(Int32, '/left_ticks', self.left_ticks_callback, 10)
        self.create_subscription(Int32, '/right_ticks', self.right_ticks_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

    def left_ticks_callback(self, msg):
        self.left_ticks = msg.data

    def right_ticks_callback(self, msg):
        self.right_ticks = msg.data

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Calculează poziția și publică odometria
        pass  # Implementare completă discutată anterior

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
