"""Encoder-relative chassis motion; calibrated geometry and loaded stop are required."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 与bench/base互斥运行；启动只核验配置，不自动使能。
    config = PathJoinSubstitution([FindPackageShare('robot_wheel_control'), 'config', 'robot_wheel_control.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config),
        DeclareLaunchArgument('backend', default_value='direct_usb_sdk', choices=['direct_usb_sdk', 'socketcan']),
        Node(package='robot_wheel_control', executable='wheel_bench_node', output='screen',
             parameters=[{'config_file': LaunchConfiguration('config_file'),
                          'backend': LaunchConfiguration('backend'), 'operation_mode': 'relative'}]),
    ])
