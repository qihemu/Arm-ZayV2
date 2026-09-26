"""Live perception only. Deliberately has no velocity publisher or wheel service client."""
import copy
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException
from robot_interfaces.msg import LidarState
from robot_interfaces.srv import GetLidarState
from lidar_slam.algorithms import scan_points, transform_points, clearance


class ScanProcessor(Node):
    def __init__(self):
        super().__init__('scan_processor', namespace='lidar')
        # Geometry stays uncalibrated until a measured configuration is supplied.
        defaults = {'base_frame': 'base_link', 'sensor_frame': 'laser_frame',
                    'scan_timeout_s': 0.35, 'future_tolerance_s': 0.02,
                    'min_valid_points': 20, 'min_span_rad': 6.0,
                    'geometry_calibrated': False, 'footprint_bounds': [0.0, 0.0, 0.0, 0.0],
                    'stop_margin_m': 0.20, 'caution_margin_m': 0.50}
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        c = self.cfg
        scalars = [c[k] for k in ('scan_timeout_s', 'future_tolerance_s', 'min_span_rad',
                                  'stop_margin_m', 'caution_margin_m')]
        if not all(math.isfinite(v) for v in scalars):
            raise ValueError('non-finite configuration')
        if not (0 < c['scan_timeout_s'] <= 2 and 0 <= c['future_tolerance_s'] <= 0.1
                and 2 <= c['min_valid_points'] <= 20000 and 0 < c['min_span_rad'] <= 2*math.pi
                and 0 <= c['stop_margin_m'] < c['caution_margin_m']):
            raise ValueError('invalid scan timing, coverage, or margins')
        bounds = c['footprint_bounds']
        if len(bounds) != 4 or not all(math.isfinite(v) for v in bounds):
            raise ValueError('footprint_bounds must be finite [xmin,xmax,ymin,ymax]')
        if c['geometry_calibrated'] and not (bounds[0] < 0 < bounds[1] and bounds[2] < 0 < bounds[3]):
            raise ValueError('calibrated footprint must surround base_link')
        # No runtime parameter changes: calibration changes require a clean restart.
        from rcl_interfaces.msg import SetParametersResult
        self.add_on_set_parameters_callback(lambda _: SetParametersResult(
            successful=False, reason='restart node with calibrated configuration'))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, 'points', qos_profile_sensor_data)
        self.obstacle_pub = self.create_publisher(PointCloud2, 'obstacles', qos_profile_sensor_data)
        self.state_pub = self.create_publisher(LidarState, 'state', 1)
        self.sub = self.create_subscription(LaserScan, 'scan_raw', self.on_scan, qos_profile_sensor_data)
        self.service = self.create_service(GetLidarState, 'get_state', self.get_state)
        self.state = LidarState(level=LidarState.UNKNOWN, reason='no scan received',
                                nearest_clearance_m=float('nan'), scan_age_s=float('inf'))
        self.received = None
        self.last_stamp = 0
        self.last_clock = 0
        # A paused /clock must not freeze the no-data watchdog.
        self.timer = self.create_timer(0.05, self.publish_state, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_scan(self, scan):
        c = self.cfg
        state = LidarState(level=LidarState.UNKNOWN, scan_stamp=scan.header.stamp,
                           geometry_calibrated=c['geometry_calibrated'],
                           nearest_clearance_m=float('nan'))
        self.state = state
        try:
            stamp = Time.from_msg(scan.header.stamp).nanoseconds
            now_ns = self.get_clock().now().nanoseconds
            clock_rewound = now_ns < self.last_clock
            self.last_clock = now_ns
            age = (now_ns - stamp) / 1e9
            if stamp <= 0 or age < -c['future_tolerance_s'] or age > c['scan_timeout_s']:
                raise ValueError('scan timestamp is stale, zero or in the future')
            if clock_rewound:
                # Only an actual ROS clock rewind resets ordering; network reordering does not.
                self.last_stamp = stamp
                raise ValueError('clock rewind; waiting for next increasing scan')
            if stamp <= self.last_stamp:
                raise ValueError('duplicate/out-of-order scan')
            self.last_stamp = stamp
            if scan.header.frame_id != c['sensor_frame']:
                raise ValueError('unexpected scan frame')
            points, ranges = scan_points(scan)
            state.valid_points = len(points)
            clean = copy.deepcopy(scan)
            clean.ranges = ranges
            if len(clean.intensities) not in (0, len(ranges)):
                clean.intensities = []
            self.scan_pub.publish(clean)
            self.cloud_pub.publish(create_cloud_xyz32(scan.header, points))
            if len(points) < c['min_valid_points'] or scan.angle_max - scan.angle_min < c['min_span_rad']:
                raise ValueError('insufficient valid returns or angular span')
            state.scan_valid = True
            tf = self.buffer.lookup_transform(c['base_frame'], scan.header.frame_id,
                                              Time.from_msg(scan.header.stamp)).transform
            t, q = tf.translation, tf.rotation
            transformed = transform_points(points, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
            state.transform_valid = True
            header = Header(stamp=scan.header.stamp, frame_id=c['base_frame'])
            self.obstacle_pub.publish(create_cloud_xyz32(header, transformed))
            if not c['geometry_calibrated']:
                raise ValueError('mount and full robot footprint are not calibrated')
            state.nearest_clearance_m = clearance(transformed, c['footprint_bounds'])
            distance = state.nearest_clearance_m
            state.level = (LidarState.STOP if distance <= c['stop_margin_m'] else
                           LidarState.CAUTION if distance <= c['caution_margin_m'] else LidarState.CLEAR)
            state.reason = 'scan-plane observation only; not a motion permit'
        except (ValueError, TransformException) as error:
            state.reason = str(error)
        self.received = time.monotonic()
        self.publish_state()

    def publish_state(self):
        state = self.state
        now = self.get_clock().now()
        state.header = Header(stamp=now.to_msg(), frame_id=self.cfg['base_frame'])
        age = (now.nanoseconds - Time.from_msg(state.scan_stamp).nanoseconds) / 1e9
        state.scan_age_s = float(age) if self.received is not None else float('inf')
        expired = self.received is None or time.monotonic() - self.received > self.cfg['scan_timeout_s']
        if expired or age > self.cfg['scan_timeout_s'] or age < -self.cfg['future_tolerance_s']:
            state.level = LidarState.UNKNOWN
            state.scan_valid = False
            state.transform_valid = False
            state.nearest_clearance_m = float('nan')
            state.reason = 'no fresh scan / clock discontinuity'
        self.state_pub.publish(state)

    def get_state(self, request, response):
        self.publish_state()
        response.state = self.state
        return response


def main():
    rclpy.init()
    node = ScanProcessor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
