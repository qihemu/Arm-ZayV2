"""C1 acquisition/processing. Never starts arm, wheel, or navigation controllers."""
import math
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from lidar_slam.configuration import config_path, load_configuration


def setup(context):
    share = Path(get_package_share_directory('lidar_slam'))
    value = lambda name: LaunchConfiguration(name).perform(context)
    enabled = lambda name: value(name).lower() == 'true'
    sim = enabled('use_sim_time')
    configuration = load_configuration(value('config_file'))
    actions = []
    # Local replay/external-driver mode uses the exact same processors, without serial access.
    if enabled('driver'):
        if sim:
            raise ValueError('real C1 driver cannot use replay time; set driver:=false')
        overrides = {'use_sim_time': False}
        if value('serial_port'):
            overrides['serial_port'] = value('serial_port')
        actions.append(Node(package='lidar_slam', executable='c1_driver', namespace='lidar',
                            name='c1_driver', output='screen',
                            parameters=[configuration['driver'], overrides]))
    if enabled('publish_mount_tf'):
        mount = configuration['mount']
        xyz, rpy = mount['xyz_m'], mount['rpy_rad']
        if mount['calibrated'] is not True or len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(v) for v in xyz + rpy):
            raise ValueError('mount file must contain calibrated finite XYZ/RPY')
        args = []
        for key, v in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), xyz + rpy):
            args.extend(['--' + key, str(v)])
        args.extend(['--frame-id', 'base_link', '--child-frame-id', 'laser_frame'])
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            name='lidar_mount_tf', arguments=args, parameters=[{'use_sim_time': sim}]))
    data_overrides = {'use_sim_time': sim}
    if value('data_root'):
        data_overrides['data_root'] = value('data_root')
    actions.extend([
        Node(package='lidar_slam', executable='scan_processor', namespace='lidar',
             parameters=[configuration['perception'], {'use_sim_time': sim}], output='screen'),
        Node(package='lidar_slam', executable='data_manager', namespace='lidar',
             parameters=[configuration['storage'], data_overrides], output='screen')])
    if enabled('wheel_odom_tf'):
        actions.append(Node(package='lidar_slam', executable='wheel_odom_tf',
                            parameters=[{'use_sim_time': sim}], output='screen'))
    if enabled('rviz'):
        actions.append(Node(package='rviz2', executable='rviz2',
                            arguments=['-d', str(share / 'rviz/c1.rviz')],
                            parameters=[{'use_sim_time': sim}]))
    return actions


def generate_launch_description():
    arguments = {'serial_port': '', 'driver': 'true', 'rviz': 'true',
                 'use_sim_time': 'false', 'publish_mount_tf': 'false', 'wheel_odom_tf': 'false',
                 'config_file': config_path(),
                 'data_root': os.environ.get('LIDAR_SLAM_DATA_ROOT', '')}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in arguments.items()] + [OpaqueFunction(function=setup)])
