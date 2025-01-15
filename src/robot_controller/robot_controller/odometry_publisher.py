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

    # Calculăm diferențele de tick-uri
    delta_left_ticks = self.left_ticks
    delta_right_ticks = self.right_ticks

    # Calculăm distanțele parcurse de fiecare roată
    left_distance = delta_left_ticks * self.distance_per_tick
    right_distance = delta_right_ticks * self.distance_per_tick

    # Calculăm delta-urile
    delta_distance = (left_distance + right_distance) / 2
    delta_theta = (right_distance - left_distance) / self.wheel_base

    # Actualizăm poziția globală
    self.x += delta_distance * cos(self.theta)
    self.y += delta_distance * sin(self.theta)
    self.theta += delta_theta

    # Normalizează theta
    self.theta = (self.theta + pi) % (2 * pi) - pi

    # Creăm mesajul Odometry
    odom_msg = Odometry()
    odom_msg.header.stamp = current_time.to_msg()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    # Setăm poziția
    odom_msg.pose.pose.position.x = self.x
    odom_msg.pose.pose.position.y = self.y
    odom_msg.pose.pose.orientation.z = sin(self.theta / 2)
    odom_msg.pose.pose.orientation.w = cos(self.theta / 2)

    # Publicăm odometria
    self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
