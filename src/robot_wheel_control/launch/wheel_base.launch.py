"""Independent ros2_control chassis process; unified YAML generates control config."""
import os
import tempfile
import xml.sax.saxutils as xml
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def setup(context):
    path = LaunchConfiguration('config_file').perform(context)
    backend = LaunchConfiguration('backend').perform(context)
    with open(path, encoding='utf8') as source:
        c = yaml.safe_load(source)['robot_wheel_control']
    if c['operation_mode'] != 'base':
        raise RuntimeError('Set operation_mode: base in the source YAML after commissioning')
    wheels = [c['wheels'][name] for name in ('left', 'right')]
    radii = [wheel['effective_radius_m'] for wheel in wheels]
    separation = c['geometry']['wheel_separation_m']
    if not separation or not all(radii):
        raise RuntimeError('Real base launch requires calibrated wheel radii/separation; use bench first')
    joints = [wheel['joint_name'] for wheel in wheels]
    # 此URDF只含底盘控制资源；不加载机械臂标签，不发布整车TF。
    hardware = ''.join(f'<joint name={xml.quoteattr(name)}><command_interface name="velocity"/>'
                       '<state_interface name="position"/><state_interface name="velocity"/></joint>' for name in joints)
    physical = ''.join(f'<link name="wheel_{i}"/><joint name={xml.quoteattr(name)} type="continuous">'
                       f'<parent link="base_link"/><child link="wheel_{i}"/><axis xyz="0 1 0"/></joint>'
                       for i, name in enumerate(joints))
    description = (f'<robot name="h55_base"><link name="base_link"/>{physical}'
                   '<ros2_control name="H55Base" type="system"><hardware>'
                   '<plugin>robot_wheel_control/H55BaseHardware</plugin>'
                   f'<param name="config_file">{xml.escape(path)}</param>'
                   f'<param name="backend">{xml.escape(backend)}</param></hardware>'
                   f'{hardware}</ros2_control></robot>')
    limits = c['limits']
    controllers = {
        '/base/controller_manager': {'ros__parameters': {
            'update_rate': int(c['timing']['control_rate_hz']),
            'diff_drive_controller': {'type': 'diff_drive_controller/DiffDriveController'},
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'}}},
        '/base/diff_drive_controller': {'ros__parameters': {
            'left_wheel_names': [joints[0]], 'right_wheel_names': [joints[1]],
            'wheel_separation': float(separation), 'wheel_radius': float(radii[0]),
            'left_wheel_radius_multiplier': 1.0, 'right_wheel_radius_multiplier': float(radii[1] / radii[0]),
            'publish_rate': float(c['timing']['odom_publish_rate_hz']),
            'base_frame_id': 'base_link', 'odom_frame_id': 'odom',
            'tf_frame_prefix_enable': False, 'enable_odom_tf': False,
            'open_loop': False, 'position_feedback': True, 'use_stamped_vel': True,
            'cmd_vel_timeout': c['timing']['command_timeout_ms'] / 1000.0,
            'linear.x.has_velocity_limits': True,
            'linear.x.max_velocity': float(limits['max_linear_speed_m_s']),
            'linear.x.min_velocity': -float(limits['max_linear_speed_m_s']),
            'angular.z.has_velocity_limits': True,
            'angular.z.max_velocity': float(limits['max_angular_speed_rad_s']),
            'angular.z.min_velocity': -float(limits['max_angular_speed_rad_s']),
            'linear.x.has_acceleration_limits': True,
            'linear.x.max_acceleration': float(limits['max_linear_acceleration_m_s2']),
            'linear.x.min_acceleration': -float(limits['max_linear_deceleration_m_s2']),
            'angular.z.has_acceleration_limits': True,
            'angular.z.max_acceleration': float(limits['max_angular_acceleration_rad_s2']),
            'angular.z.min_acceleration': -float(limits['max_angular_deceleration_rad_s2']),
        }},
        '/base/joint_state_broadcaster': {'ros__parameters': {'use_local_topics': True}},
    }
    fd, controller_path = tempfile.mkstemp(prefix='h55-controllers-', suffix='.yaml')
    with os.fdopen(fd, 'w') as target:
        yaml.safe_dump(controllers, target)
    # 临时文件由本次launch拥有，退出清理；源配置从不回写。
    def cleanup(_context):
        if os.path.exists(controller_path):
            os.unlink(controller_path)
        return []
    return [
        Node(package='controller_manager', executable='ros2_control_node', namespace='/base',
             parameters=[controller_path, {'robot_description': description}], output='screen',
             remappings=[('/base/diff_drive_controller/odom', '/base/wheel_odom'),
                         ('/base/joint_state_broadcaster/joint_states', '/base/joint_states')]),
        Node(package='controller_manager', executable='spawner',
             arguments=['diff_drive_controller', 'joint_state_broadcaster', '-c', '/base/controller_manager'],
             output='screen'),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([
            FindPackageShare('robot_wheel_control'), 'config', 'robot_wheel_control.yaml'])),
        DeclareLaunchArgument('backend', default_value='', choices=['', 'direct_usb_sdk', 'socketcan']),
        OpaqueFunction(function=setup),
    ])
