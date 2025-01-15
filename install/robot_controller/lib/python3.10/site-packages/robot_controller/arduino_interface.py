import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Quaternion
import serial
import math

class ArduinoInterfaceNode(Node):
    def __init__(self):
        super().__init__('arduino_interface_node')

        # Configurație serială
        self.serial_port = '/dev/ttyUSB0'  # Schimbă cu portul Arduino
        self.baudrate = 115200
        self.arduino_serial = serial.Serial(self.serial_port, self.baudrate, timeout=1)

        # Publicatori
        self.lidar_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)

        # Parametrii pentru Lidar
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(1)
        self.range_min = 0.1
        self.range_max = 12.0
        self.ranges = [float('inf')] * 360

        # Timer pentru procesare
        self.timer = self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        try:
            if self.arduino_serial.in_waiting > 0:
                line = self.arduino_serial.readline().decode('utf-8').strip()
                data = line.split(',')
                if len(data) == 7:
                    distance = float(data[0])
                    angle = float(data[1])
                    signal_strength = float(data[2])
                    qx, qy, qz, qw = map(float, data[3:])

                    # Publică datele
                    self.publish_lidar(distance, angle)
                    self.publish_imu(qx, qy, qz, qw)
        except Exception as e:
            self.get_logger().error(f"Eroare citire serial: {e}")

    def publish_lidar(self, distance, angle):
        angle_index = int((angle + 180) % 360)
        if self.range_min <= distance <= self.range_max:
            self.ranges[angle_index] = distance

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "laser_frame"
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        scan_msg.ranges = self.ranges
        self.lidar_publisher.publish(scan_msg)
        self.ranges = [float('inf')] * 360
        
    def publish_imu(self, qx, qy, qz, qw):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_frame"
        imu_msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        self.imu_publisher.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoInterfaceNode()
    rclpy.spin(node)
    rclpy.shutdown()
