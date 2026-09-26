"""Immutable versioned artifacts. Standard files plus explicit provenance, never in install/."""
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile

from PIL import Image
import yaml

MAX_FILE_BYTES = 128 * 1024 * 1024


def identifier(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}', value) or value in ('.', '..'):
        raise ValueError('use 1..80 ASCII letters/digits/_.- for artifact identifiers')
    return value


def digest(path):
    if not path.is_file() or path.stat().st_size > MAX_FILE_BYTES:
        raise ValueError('missing file or file exceeds 128 MiB limit')
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def child(root, relative):
    """Reject traversal and symlink escape for all catalog lookups."""
    root = Path(root).expanduser().resolve()
    path = (root / relative).resolve()
    if not path.is_relative_to(root) or path == root:
        raise ValueError('path escapes artifact root')
    return path


@contextmanager
def transaction(destination):
    """Only expose a complete bundle; serialize writers and forbid overwrite."""
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with (destination.parent / '.writer.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if destination.exists():
            raise FileExistsError('artifact exists; choose a new revision/name')
        staging = Path(tempfile.mkdtemp(prefix='.pending-', dir=destination.parent))
        try:
            yield staging
            # fsync artifacts and directory before publication; rename stays on one filesystem.
            for path in staging.iterdir():
                if path.is_file():
                    with path.open('rb') as stream:
                        os.fsync(stream.fileno())
            fd = os.open(staging, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
            os.rename(staging, destination)
            fd = os.open(destination.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        finally:
            if staging.exists():
                shutil.rmtree(staging)


def manifest(folder, metadata):
    data = dict(metadata, schema_version=1, created_utc=datetime.now(timezone.utc).isoformat())
    data['files'] = {p.name: digest(p) for p in folder.iterdir() if p.is_file()}
    (folder / 'manifest.json').write_text(json.dumps(data, indent=4, ensure_ascii=False) + '\n')


def verify(folder):
    folder = Path(folder)
    if (folder / 'manifest.json').stat().st_size > 1024 * 1024:
        raise ValueError('oversized manifest')
    data = json.loads((folder / 'manifest.json').read_text())
    if data.get('schema_version') != 1 or not data.get('files'):
        raise ValueError('unsupported or empty manifest')
    for name, expected in data['files'].items():
        if Path(name).name != name or digest(child(folder, name)) != expected:
            raise ValueError('artifact hash mismatch or unsafe path: ' + name)
    return data


def save_cloud(root, name, points, frame, stamp):
    """PCD ASCII XYZ is a portable single-scan export, not a registered 3D map."""
    if not frame or not points or len(points) > 20000:
        raise ValueError('empty frame/cloud or too many points')
    if not all(len(p) == 3 and all(math.isfinite(float(v)) for v in p) for p in points):
        raise ValueError('PCD requires finite XYZ coordinates')
    target = child(root, 'clouds/' + identifier(name))
    with transaction(target) as stage:
        header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
                  f'COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                  f'POINTS {len(points)}\nDATA ascii\n')
        (stage / 'cloud.pcd').write_text(header + ''.join(' '.join(f'{v:.9g}' for v in p) + '\n' for p in points))
        manifest(stage, {'kind': 'sensor_snapshot', 'frame_id': frame, 'stamp_ns': stamp,
                         'point_count': len(points), 'registered_map': False})
    return target


def load_cloud(root, name):
    folder = child(root, 'clouds/' + identifier(name))
    meta = verify(folder)
    if meta.get('kind') != 'sensor_snapshot' or 'cloud.pcd' not in meta['files']:
        raise ValueError('not a supported sensor snapshot')
    lines = (folder / 'cloud.pcd').read_text().splitlines()
    header, data_at = {}, None
    for i, line in enumerate(lines):
        if not line or line.startswith('#'):
            continue
        key, _, value = line.partition(' ')
        header[key] = value.strip()
        if key == 'DATA':
            data_at = i + 1
            break
    expected = {'FIELDS': 'x y z', 'SIZE': '4 4 4', 'TYPE': 'F F F', 'COUNT': '1 1 1',
                'DATA': 'ascii', 'HEIGHT': '1'}
    if data_at is None or any(header.get(k) != v for k, v in expected.items()):
        raise ValueError('only package-generated ASCII XYZ PCD is supported')
    count = int(header['POINTS'])
    if not 0 < count <= 20000 or int(header['WIDTH']) != count or meta['point_count'] != count:
        raise ValueError('invalid PCD count')
    points = [tuple(map(float, line.split())) for line in lines[data_at:] if line.strip()]
    if len(points) != count or any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
        raise ValueError('corrupt PCD points')
    return meta, points


def import_map(root, site, floor, revision, source_yaml, posegraph='', context=''):
    """Import completed SLAM exports as a new immutable floor revision."""
    ids = [identifier(v) for v in (site, floor, revision)]
    source = Path(source_yaml).expanduser().resolve()
    digest(source)
    info = yaml.safe_load(source.read_text())
    if not isinstance(info, dict):
        raise ValueError('map YAML must contain a mapping')
    resolution = float(info['resolution'])
    origin = info['origin']
    if not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3 or not all(math.isfinite(v) for v in origin):
        raise ValueError('invalid map resolution or origin')
    if info.get('negate') not in (0, 1) or info.get('mode', 'trinary') not in ('trinary', 'scale', 'raw'):
        raise ValueError('invalid map mode or negate')
    if not 0 <= float(info['free_thresh']) < float(info['occupied_thresh']) <= 1:
        raise ValueError('invalid map thresholds')
    image = Path(info['image'])
    if image.is_absolute():
        raise ValueError('map image must be relative to map YAML')
    image = child(source.parent, image)
    digest(image)
    if image.suffix.lower() not in ('.pgm', '.png'):
        raise ValueError('only PGM/PNG map images supported')
    with Image.open(image) as im:
        width, height = im.size
        im.verify()
    graph_files = []
    if posegraph:
        graph_files = [Path(str(Path(posegraph).expanduser()) + suffix) for suffix in ('.posegraph', '.data')]
        for path in graph_files:
            digest(path)
    metadata = {}
    if context:
        path = Path(context).expanduser()
        digest(path)
        metadata = yaml.safe_load(path.read_text())
        if not isinstance(metadata, dict):
            raise ValueError('context YAML must be a mapping')
    target = child(root, 'maps/' + '/'.join(ids))
    with transaction(target) as stage:
        output_image = 'map' + image.suffix.lower()
        shutil.copy2(image, stage / output_image)
        info['image'] = output_image
        (stage / 'map.yaml').write_text(yaml.safe_dump(info, sort_keys=False))
        for path, suffix in zip(graph_files, ('.posegraph', '.data')):
            shutil.copy2(path, stage / ('graph' + suffix))
        manifest(stage, {'kind': 'floor_map', 'site_id': site, 'floor_id': floor,
                         'revision': revision, 'map_frame': 'map', 'width': width, 'height': height,
                         'resolution': resolution, 'origin': origin, 'has_posegraph': bool(graph_files),
                         'source_yaml': str(source), 'context': metadata,
                         'validation': 'imported; localization and field acceptance pending'})
    return target
