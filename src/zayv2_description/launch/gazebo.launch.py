from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Start Gazebo Classic with the installed URDF and publish its frame tree.
    package_share = Path(get_package_share_directory('zayv2_description'))
    urdf_path = package_share / 'urdf' / 'zayv2_description.urdf'
    robot_description = urdf_path.read_text()
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py',
    ])

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch)),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'base_footprint',
            ],
            output='screen',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'zayv2_description', '-file', str(urdf_path)],
            output='screen',
        ),
    ])
