import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sin, cos, pi
import serial
import threading

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')

        # Parametrii robotului
        self.wheel_base = 0.3  # Distanța între roți (metri)
        self.wheel_radius = 0.05  # Raza roții (metri)
        self.ticks_per_revolution = 1000  # Număr tick-uri pe rotație

        # Calculăm distanța per tick
        self.distance_per_tick = (2 * pi * self.wheel_radius) / self.ticks_per_revolution

        # Configurație serială
        self.serial_port = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.motor_serial = serial.Serial(self.serial_port, self.baudrate, timeout=1)

        # Subscriberi
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publicatori
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Variabile odometrie
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Thread pentru citirea serială
        self.read_thread = threading.Thread(target=self.read_encoder_data)
        self.read_thread.daemon = True
        self.read_thread.start()

    def cmd_vel_callback(self, msg):
        """
        Procesează comenzile /cmd_vel și trimite la Arduino.
        """
        linear = msg.linear.x
        angular = msg.angular.z

        # Calculăm vitezele pentru fiecare motor
        left_speed = int((linear - angular * self.wheel_base / 2) * 255)
        right_speed = int((linear + angular * self.wheel_base / 2) * 255)

        # Trimitere comandă la Arduino
        command = f"L{left_speed},R{right_speed}\n"
        self.motor_serial.write(command.encode())
        self.get_logger().info(f"Sent command: {command.strip()}")

    def read_encoder_data(self):
        """
        Citește datele encoderelor de la Arduino.
        """
        while rclpy.ok():
            try:
                if self.motor_serial.in_waiting > 0:
                    line = self.motor_serial.readline().decode('utf-8').strip()
                    if line.startswith("E"):
                        data = line[1:].split(',')
                        if len(data) == 2:
                            self.left_ticks = int(data[0])
                            self.right_ticks = int(data[1])
                            self.update_odometry()
            except Exception as e:
                self.get_logger().error(f"Error reading encoder data: {e}")

    def update_odometry(self):
        """
        Calculează odometria și publică pe topicul /odom.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Calculăm diferențele de tick-uri
        delta_left_ticks = self.left_ticks - self.last_left_ticks
        delta_right_ticks = self.right_ticks - self.last_right_ticks

        # Actualizăm valorile pentru următoarea iterație
        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks

        # Calculăm distanțele parcurse de fiecare roată
        left_distance = delta_left_ticks * self.distance_per_tick
        right_distance = delta_right_ticks * self.distance_per_tick

        # Calculăm delta-urile
        delta_distance = (left_distance + right_distance) / 2
        delta_theta = (right_distance - left_distance) / self.wheel_base

        # Actualizăm poziția
        self.x += delta_distance * cos(self.theta)
        self.y += delta_distance * sin(self.theta)
        self.theta += delta_theta

        # Normalizăm theta
        self.theta = (self.theta + pi) % (2 * pi) - pi

        # Publicăm odometria
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = cos(self.theta / 2)
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
