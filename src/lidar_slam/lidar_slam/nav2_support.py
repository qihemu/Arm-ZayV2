"""Generate a Nav2 perception fragment from the single source of chassis geometry."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import yaml
from lidar_slam.configuration import config_path, load_configuration


def render(template, parameters, allow_uncalibrated=False):
    bounds = parameters['footprint_bounds']
    if len(bounds) != 4 or not all(math.isfinite(v) for v in bounds):
        raise ValueError('invalid footprint bounds')
    xmin, xmax, ymin, ymax = bounds
    if not xmin < 0 < xmax or not ymin < 0 < ymax:
        raise ValueError('footprint must surround base_link')
    if not parameters['geometry_calibrated'] and not allow_uncalibrated:
        raise ValueError('geometry uncalibrated; use --preview only for an offline fragment')
    margin = parameters['stop_margin_m']
    timeout = parameters['scan_timeout_s']
    if not math.isfinite(margin) or margin < 0 or not math.isfinite(timeout) or not 0 < timeout <= 2:
        raise ValueError('invalid stop margin or scan timeout')
    if parameters['base_frame'] != 'base_link':
        raise ValueError('Nav2 contract currently requires base_link')
    result = copy.deepcopy(template)
    footprint = [[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]]
    # Nav2 costmap footprint is a serialized polygon string; Humble collision points are flat doubles.
    for name in ('local_costmap', 'global_costmap'):
        result[name][name]['ros__parameters']['footprint'] = json.dumps(footprint)
    result['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['scan']['expected_update_rate'] = timeout
    collision = result['collision_monitor']['ros__parameters']
    collision['source_timeout'] = timeout
    collision['StopZone']['points'] = [xmin-margin, ymin-margin, xmax+margin, ymin-margin,
                                       xmax+margin, ymax+margin, xmin-margin, ymax+margin]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config-file', default=config_path())
    parser.add_argument('--output', required=True)
    parser.add_argument('--preview', action='store_true', help='allow uncalibrated offline export; grants no motion authority')
    args = parser.parse_args()
    try:
        configuration = load_configuration(args.config_file)
        source = Path(args.config_file).read_bytes()
        result = render(configuration['nav2'], configuration['perception'], args.preview)
        output = Path(args.output).expanduser()
        # Never replace a hand-edited navigation config or the canonical source file.
        with output.open('x') as stream:
            stream.write('# Perception fragment only; no controller or motion authorization.\n')
            stream.write(f'# Preview: {args.preview}; config_sha256: {hashlib.sha256(source).hexdigest()}\n')
            yaml.safe_dump(result, stream, sort_keys=False, indent=4)
        print(output.resolve())
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        parser.exit(1, f'Nav2 export rejected: {error}\n')
