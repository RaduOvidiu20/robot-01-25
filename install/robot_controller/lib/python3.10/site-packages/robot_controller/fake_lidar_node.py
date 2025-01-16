import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class FakeLidarNode(Node):
    def __init__(self):
        super().__init__('fake_lidar_node')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)  # Publish every 100ms
        self.get_logger().info("Fake Lidar Node for 4m Square Started")

    def publish_scan(self):
        # Configure scan parameters
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser_frame"
        scan.angle_min = -math.pi  # -180 degrees
        scan.angle_max = math.pi   # +180 degrees
        scan.angle_increment = math.radians(1)  # 1 degree per step
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1  # Minimum detectable range
        scan.range_max = 2.0  # Maximum detectable range

        # Initialize ranges to 'inf' (no obstacle detected)
        ranges = [float('inf')] * 360

        # Simulate walls at a 4-meter square
        for angle in range(360):
            rad = math.radians(angle)
            x = 2.0 * math.cos(rad)  # X-coordinate of the point
            y = 2.0 * math.sin(rad)  # Y-coordinate of the point

            # Check if the point falls on the square walls
            if (-2.0 <= x <= 2.0 and (abs(y - 2.0) < 0.1 or abs(y + 2.0) < 0.1)) or \
               (-2.0 <= y <= 2.0 and (abs(x - 2.0) < 0.1 or abs(x + 2.0) < 0.1)):
                ranges[angle] = 2.0  # Set distance to 4 meters (wall)

        scan.ranges = ranges
        self.publisher_.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarNode()
    rclpy.spin(node)
    rclpy.shutdown()
