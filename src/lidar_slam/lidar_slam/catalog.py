"""Offline catalog operations never launch ROS nodes or change the active floor."""
import argparse
import json
import os
from pathlib import Path
from lidar_slam.storage import import_map, verify, child, identifier


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', default=os.environ.get('LIDAR_SLAM_DATA_ROOT', str(Path.home() / '.local/share/arm_zay/lidar_slam')))
    commands = parser.add_subparsers(dest='command', required=True)
    add = commands.add_parser('import')
    for name in ('site', 'floor', 'revision', 'map-yaml'):
        add.add_argument('--' + name, required=True)
    add.add_argument('--posegraph-prefix', default='')
    add.add_argument('--context', default='', help='YAML calibration, software, bag and acceptance provenance')
    commands.add_parser('list')
    check = commands.add_parser('verify')
    check.add_argument('site')
    check.add_argument('floor')
    check.add_argument('revision')
    args = parser.parse_args()
    try:
        if args.command == 'import':
            result = import_map(args.root, args.site, args.floor, args.revision,
                                args.map_yaml, args.posegraph_prefix, args.context)
            print(result)
        elif args.command == 'list':
            for path in sorted(Path(args.root).expanduser().glob('maps/*/*/*/manifest.json')):
                print(json.dumps(verify(path.parent), ensure_ascii=False))
        else:
            folder = child(args.root, 'maps/' + '/'.join(identifier(v) for v in (args.site, args.floor, args.revision)))
            print(json.dumps(verify(folder), ensure_ascii=False, indent=4))
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f'catalog error: {error}\n')
