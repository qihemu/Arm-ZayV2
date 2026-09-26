"""Pure scan geometry: no ROS, device access, or motion commands."""
import math


def scan_points(scan, max_points=20000):
    """Validate geometry before indexing; unknown returns remain NaN, never free space."""
    n = len(scan.ranges)
    values = (scan.angle_min, scan.angle_max, scan.angle_increment,
              scan.range_min, scan.range_max, scan.scan_time, scan.time_increment)
    if not 2 <= n <= max_points or not all(math.isfinite(v) for v in values):
        raise ValueError('invalid scan size or non-finite metadata')
    if scan.angle_increment <= 0 or not 0 <= scan.range_min < scan.range_max:
        raise ValueError('invalid angle increment or range bounds')
    if scan.scan_time < 0 or scan.time_increment < 0:
        raise ValueError('negative scan timing')
    if abs(scan.angle_min + (n - 1) * scan.angle_increment - scan.angle_max) > max(0.01, 2 * scan.angle_increment):
        raise ValueError('inconsistent scan angle span')
    if scan.angle_max - scan.angle_min > 2 * math.pi + 0.05:
        raise ValueError('scan spans more than one revolution')
    points, ranges = [], []
    for i, distance in enumerate(scan.ranges):
        if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
            angle = scan.angle_min + i * scan.angle_increment
            points.append((distance * math.cos(angle), distance * math.sin(angle), 0.0))
            ranges.append(distance)
        else:
            ranges.append(float('nan'))
    return points, ranges


def transform_points(points, translation, quaternion):
    """Apply a full rigid transform, including tilted sensor mounting."""
    x, y, z, w = quaternion
    if not all(math.isfinite(v) for v in (*translation, x, y, z, w)):
        raise ValueError('non-finite transform')
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if abs(norm - 1.0) > 0.01:
        raise ValueError('invalid transform quaternion')
    x, y, z, w = (v / norm for v in (x, y, z, w))
    matrix = ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
              (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
              (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))
    return [tuple(sum(row[i]*p[i] for i in range(3)) + t
                  for row, t in zip(matrix, translation)) for p in points]


def clearance(points, bounds):
    """Conservative XY distance to measured full-robot rectangle, including arms."""
    xmin, xmax, ymin, ymax = bounds
    return min((math.hypot(max(xmin-x, 0.0, x-xmax), max(ymin-y, 0.0, y-ymax))
                for x, y, _ in points), default=float('nan'))
