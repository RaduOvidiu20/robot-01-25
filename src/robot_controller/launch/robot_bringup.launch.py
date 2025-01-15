from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_controller', executable='arduino_interface.py', name='arduino_interface'),
        Node(package='robot_controller', executable='motor_controller.py', name='motor_controller'),
        Node(package='robot_controller', executable='odometry_publisher.py', name='odometry_publisher'),
        Node(package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox'),
        Node(package='nav2_bringup', executable='bringup_launch.py', name='navigation'),
    ])
