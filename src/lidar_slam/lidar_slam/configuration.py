"""One editable workspace config root, with an installed-resource fallback."""
import os
import argparse
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory


def config_path(filename='lidar_slam.yaml'):
    # An explicit root is authoritative: fail visibly rather than silently loading stale defaults.
    root = os.environ.get('ARM_ZAY_CONFIG_DIR')
    directory = Path(root).expanduser() if root else Path(get_package_share_directory('lidar_slam')) / 'config'
    path = directory / filename
    if not path.is_file():
        raise FileNotFoundError(f'configuration missing: {path}')
    return str(path.resolve())


def load_configuration(path):
    """Validate the container, then pass only the selected module to each ROS node."""
    data = yaml.safe_load(Path(path).expanduser().read_text())
    modules = ('driver', 'mount', 'perception', 'storage', 'slam', 'amcl',
               'record_qos', 'map_context', 'nav2')
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('expected lidar_slam configuration schema_version: 1')
    for module in modules:
        if not isinstance(data.get(module), dict):
            raise ValueError(f'missing or invalid configuration module: {module}')
    return data


def main():
    """Export tool-specific runtime artifacts without maintaining duplicate source YAMLs."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config-file', default=config_path())
    parser.add_argument('--module', choices=('record_qos', 'map_context'), required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    try:
        data = load_configuration(args.config_file)[args.module]
        output = Path(args.output).expanduser()
        with output.open('x') as stream:
            stream.write('# Generated runtime artifact; edit config/lidar_slam.yaml as the source.\n')
            yaml.safe_dump(data, stream, sort_keys=False, indent=4)
        print(output.resolve())
    except (OSError, ValueError, yaml.YAMLError) as error:
        parser.exit(1, f'configuration export rejected: {error}\n')
