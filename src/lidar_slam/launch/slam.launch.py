"""Exactly one map->odom authority: mapping, graph localization, or AMCL."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from lidar_slam.storage import verify
from lidar_slam.configuration import config_path, load_configuration


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    configuration = load_configuration(get('config_file'))
    mode, sim = get('mode'), get('use_sim_time').lower() == 'true'
    if mode == 'mapping':
        return [Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                     parameters=[configuration['slam'], {'use_sim_time': sim, 'mode': 'mapping'}], output='screen')]
    if mode not in ('localization', 'amcl'):
        raise ValueError('mode must be mapping, localization, or amcl')
    # No implicit last-map selection: a verified floor revision is always explicit.
    if not get('map_directory'):
        raise ValueError('localization needs an explicit immutable map_directory')
    directory = Path(get('map_directory')).expanduser().resolve()
    meta = verify(directory)
    if meta.get('kind') != 'floor_map':
        raise ValueError('expected a floor map bundle')
    if mode == 'localization':
        if not meta.get('has_posegraph') or not all(n in meta['files'] for n in ('graph.posegraph', 'graph.data')):
            raise ValueError('graph localization needs both serialized graph files; use amcl for grid-only maps')
        pose = [float(get(k)) for k in ('initial_x', 'initial_y', 'initial_yaw')]
        import math
        if not all(math.isfinite(v) for v in pose):
            raise ValueError('finite initial_x/initial_y/initial_yaw required for graph localization')
        return [Node(package='slam_toolbox', executable='localization_slam_toolbox_node', name='slam_toolbox',
                     parameters=[configuration['slam'], {'use_sim_time': sim, 'mode': 'localization',
                                 'map_file_name': str(directory / 'graph'), 'map_start_pose': pose}], output='screen')]
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'use_sim_time': sim, 'yaml_filename': str(directory / 'map.yaml'), 'frame_id': 'map'}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[configuration['amcl'], {'use_sim_time': sim}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='localization_lifecycle',
             parameters=[{'use_sim_time': sim, 'autostart': True, 'node_names': ['map_server', 'amcl']}])]


def generate_launch_description():
    args = {'mode': 'mapping', 'use_sim_time': 'false', 'map_directory': '',
            'config_file': config_path(),
            'initial_x': 'nan', 'initial_y': 'nan', 'initial_yaw': 'nan'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args.items()] + [OpaqueFunction(function=setup)])
