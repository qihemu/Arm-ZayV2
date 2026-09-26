"""Low-rate disk services in their own process; offline output is isolated from live obstacles."""
import os
import time
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points, create_cloud_xyz32
from std_msgs.msg import Header
from robot_interfaces.srv import SaveLidarCloud, LoadLidarReference
from lidar_slam.storage import save_cloud, load_cloud


class DataManager(Node):
    def __init__(self):
        super().__init__('data_manager', namespace='lidar')
        default = os.environ.get('LIDAR_SLAM_DATA_ROOT', str(Path.home() / '.local/share/arm_zay/lidar_slam'))
        self.root = Path(self.declare_parameter('data_root', default).value).expanduser().resolve()
        self.timeout = self.declare_parameter('snapshot_timeout_s', 0.35).value
        if not math.isfinite(self.timeout) or not 0 < self.timeout <= 2.0:
            raise ValueError('snapshot_timeout_s must be in (0, 2]')
        self.cloud, self.arrival = None, 0.0
        self.sub = self.create_subscription(PointCloud2, 'points', self.receive, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reference = self.create_publisher(PointCloud2, 'reference_points', qos)
        self.save = self.create_service(SaveLidarCloud, 'save_cloud', self.save_request)
        self.load = self.create_service(LoadLidarReference, 'load_reference', self.load_request)

    def receive(self, message):
        self.cloud, self.arrival = message, time.monotonic()

    def save_request(self, request, response):
        try:
            cloud = self.cloud
            if cloud is None:
                raise ValueError('no cloud received')
            stamp = Time.from_msg(cloud.header.stamp).nanoseconds
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if stamp <= 0 or not 0 <= age <= self.timeout or time.monotonic() - self.arrival > self.timeout:
                raise ValueError('latest cloud is stale; no snapshot written')
            points = [tuple(float(v) for v in p) for p in read_points(
                cloud, field_names=('x', 'y', 'z'), skip_nans=True)]
            directory = save_cloud(self.root, request.name, points, cloud.header.frame_id, stamp)
            response.success, response.message = True, 'single scan saved; not a registered map'
            response.directory, response.point_count = str(directory), len(points)
        except (OSError, ValueError, KeyError, TypeError) as error:
            response.message = str(error)
        return response

    def load_request(self, request, response):
        try:
            meta, points = load_cloud(self.root, request.name)
            header = Header(frame_id=meta['frame_id'], stamp=Time(nanoseconds=meta['stamp_ns']).to_msg())
            self.reference.publish(create_cloud_xyz32(header, points))
            response.success, response.message = True, 'reference only; original sensor frame and timestamp retained'
            response.point_count = len(points)
        except (OSError, ValueError, KeyError, TypeError) as error:
            response.message = str(error)
        return response


def main():
    rclpy.init()
    node = DataManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
