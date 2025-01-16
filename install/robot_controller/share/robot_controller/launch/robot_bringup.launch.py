from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_controller',
            executable='arduino_interface',
            name='arduino_interface'
        ),
        Node(
            package='robot_controller',
            executable='motor_controller',
            name='motor_controller'
        ),
        Node(
            package='robot_controller',
            executable='odometry_publisher',
            name='odometry_publisher'
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=['install/robot_controller/share/robot_controller/config/slam_toolbox_params.yaml']
        ),
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='navigation',
            parameters=['install/robot_controller/share/robot_controller/config/nav2_params.yaml']
        ),
        Node(
            package='robot_controller',
            executable='tf_broadcaster',
            name='tf_broadcaster'
        ),
    ])
