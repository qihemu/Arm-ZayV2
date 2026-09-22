from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def check_runtime_packages(context):
    # Resolve every required package before starting nodes, avoiding orphaned TF publishers.
    packages = ['robot_state_publisher', 'rviz2']
    if IfCondition(LaunchConfiguration('gui')).evaluate(context):
        packages.append('joint_state_publisher_gui')

    for package in packages:
        get_package_prefix(package)

    return []


def generate_launch_description():
    # Load the installed model so its package:// mesh paths resolve in RViz 2.
    package_share = Path(get_package_share_directory('zayv2_description'))
    robot_description = (package_share / 'urdf' / 'zayv2_description.urdf').read_text()
    rviz_config = package_share / 'rviz' / 'display.rviz'

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the joint state publisher GUI',
        ),
        OpaqueFunction(function=check_runtime_packages),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui')),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(rviz_config)],
            output='screen',
        ),
    ])
