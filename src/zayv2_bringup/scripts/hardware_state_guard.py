#!/usr/bin/env python3
"""Stop the bringup when an activated Damiao hardware component loses its active state."""

import argparse
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListHardwareComponents
from lifecycle_msgs.msg import State
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


POLL_PERIOD_SECONDS = 0.05
RESPONSE_DEADLINE_SECONDS = 0.5


class HardwareStateTracker:
    """Latch the first active state and reject every later non-active state."""

    def __init__(self, hardware_name):
        self.hardware_name = hardware_name
        self.active_seen = False

    def observe(self, components):
        matches = [component for component in components if component.name == self.hardware_name]
        if len(matches) != 1:
            raise RuntimeError(f"Expected exactly one {self.hardware_name} hardware component")

        state = matches[0].state
        if state.id == State.PRIMARY_STATE_ACTIVE:
            self.active_seen = True
        elif self.active_seen:
            raise RuntimeError(
                f"{self.hardware_name} left active state: id={state.id} label={state.label}"
            )


class HardwareStateGuard(Node):
    """Poll controller_manager and fail closed before stale states can finish a goal."""

    def __init__(self, hardware_name):
        super().__init__("damiao_hardware_state_guard")
        self._tracker = HardwareStateTracker(hardware_name)
        self._client = self.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components"
        )
        self._pending = None
        self._last_response_at = time.monotonic()
        self.create_timer(POLL_PERIOD_SECONDS, self._poll)

    def _poll(self):
        # 处理上一次异步查询，发现激活后掉线就让 launch 关闭所有节点。
        if self._pending is not None and self._pending.done():
            pending = self._pending
            self._pending = None
            try:
                response = pending.result()
                if response is None:
                    raise RuntimeError("controller_manager returned an empty hardware response")
                self._tracker.observe(response.component)
            except Exception as error:
                self.get_logger().fatal(f"Hardware state guard stopping bringup: {error}")
                raise
            self._last_response_at = time.monotonic()

        # 已使能的硬件若连续无法确认状态，也停止接受新的轨迹。
        if self._tracker.active_seen:
            silence = time.monotonic() - self._last_response_at
            if silence > RESPONSE_DEADLINE_SECONDS:
                reason = f"hardware state unavailable for {silence:.3f} s"
                self.get_logger().fatal(f"Hardware state guard stopping bringup: {reason}")
                raise RuntimeError(reason)

        if self._pending is None and self._client.service_is_ready():
            self._pending = self._client.call_async(ListHardwareComponents.Request())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware", default="DamiaoArm")
    # launch_ros 附加 --ros-args；仅解析监视器自己的参数。
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init(args=sys.argv)
    node = HardwareStateGuard(args.hardware)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # launch 正常结束时 ROS 上下文可能已由信号处理器关闭。
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
