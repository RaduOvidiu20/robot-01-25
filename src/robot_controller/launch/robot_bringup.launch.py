from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_controller_dir = get_package_share_directory('robot_controller')

    return LaunchDescription([
        # Fake Lidar node
        Node(
            package='robot_controller',
            executable='fake_lidar_node',
            name='fake_lidar_node',
            output='screen'
        ),
        # Fake Odometry node
        Node(
            package='robot_controller',
            executable='fake_odometry_node',
            name='fake_odometry_node',
            output='screen'
        ),
        # SLAM Toolbox for live mapping
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[os.path.join(robot_controller_dir, 'config', 'slam_toolbox_params.yaml')],
            output='screen'
        ),
        # TF Broadcaster
        Node(
            package='robot_controller',
            executable='tf_broadcaster',
            name='tf_broadcaster',
            output='screen'
        ),
        # Nav2 Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[os.path.join(robot_controller_dir, 'config', 'nav2_params.yaml')],
            output='screen'
        ),
    ])
