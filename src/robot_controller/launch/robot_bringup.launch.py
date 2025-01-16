from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obține directoarele necesare
    robot_controller_dir = get_package_share_directory('robot_controller')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        # Interfața cu Arduino
        Node(
            package='robot_controller',
            executable='arduino_interface',
            name='arduino_interface'
        ),
        # Controlul motoarelor
        Node(
            package='robot_controller',
            executable='motor_controller',
            name='motor_controller'
        ),
        # Publicare odometrie
        Node(
            package='robot_controller',
            executable='odometry_publisher',
            name='odometry_publisher'
        ),
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[os.path.join(robot_controller_dir, 'config', 'slam_toolbox_params.yaml')]
        ),
        # Include Nav2 Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': os.path.join(robot_controller_dir, 'config', 'map.yaml'),
                'params_file': os.path.join(robot_controller_dir, 'config', 'nav2_params.yaml'),
                'use_sim_time': 'false'
            }.items()
        ),
        # Transformări TF
        Node(
            package='robot_controller',
            executable='tf_broadcaster',
            name='tf_broadcaster'
        ),
    ])
