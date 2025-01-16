from setuptools import find_packages, setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_bringup.launch.py']),
        ('share/' + package_name + '/config', [
            'config/slam_toolbox_params.yaml',
            'config/nav2_params.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ovidiu',
    maintainer_email='ovidiu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_interface = robot_controller.arduino_interface:main',
            'motor_controller = robot_controller.motor_controller:main',
            'odometry_publisher = robot_controller.odometry_publisher:main',
            'tf_broadcaster = robot_controller.tf_broadcaster:main',
        ],
    },
)
