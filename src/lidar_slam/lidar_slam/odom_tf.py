"""Optional wheel odometry TF authority, disabled by default in launch."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from robot_interfaces.msg import WheelBaseState
from tf2_ros import TransformBroadcaster


class WheelOdomTf(Node):
    def __init__(self):
        super().__init__('wheel_odom_tf')
        self.state, self.arrival = None, 0.0
        self.last_odom = 0
        self.broadcaster = TransformBroadcaster(self)
        self.state_sub = self.create_subscription(WheelBaseState, '/base/state', self.on_state, 1)
        self.odom_sub = self.create_subscription(Odometry, '/base/wheel_odom', self.on_odom, 1)

    def on_state(self, state):
        self.state, self.arrival = state, time.monotonic()

    def on_odom(self, odom):
        # Bench mode and stale/faulted wheel feedback cannot manufacture odom TF.
        state = self.state
        if state is None or not state.odometry_valid or time.monotonic() - self.arrival > 0.25:
            return
        now = self.get_clock().now().nanoseconds
        stamps = [Time.from_msg(m.header.stamp).nanoseconds for m in (state, odom)]
        if any(t <= 0 or not 0 <= (now-t)/1e9 <= 0.25 for t in stamps):
            return
        if stamps[1] <= self.last_odom or abs(stamps[0] - stamps[1]) > 250000000:
            self.last_odom = stamps[1]
            return
        if odom.header.frame_id != 'odom' or odom.child_frame_id != 'base_link':
            return
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01:
            return
        self.last_odom = stamps[1]
        tf = TransformStamped(header=odom.header, child_frame_id='base_link')
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p.x, p.y, p.z
        tf.transform.rotation = q
        self.broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = WheelOdomTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
