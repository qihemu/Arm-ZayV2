#!/usr/bin/env python3
"""Six-axis bench slider GUI that sends measured-state-based ros2_control trajectories."""

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import signal
import sys
import time

import yaml
from PyQt5.QtCore import QSignalBlocker, Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)
import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from lifecycle_msgs.msg import State
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = tuple(f"joint{number}" for number in range(1, 7))
SLIDER_STEPS = 10000
SEND_PERIOD_SECONDS = 0.1
STATUS_TIMEOUT_SECONDS = 0.5
MIN_TRAJECTORY_SECONDS = 0.2
TRAJECTORY_TIME_MARGIN = 1.2


@dataclass(frozen=True)
class AxisLimits:
    name: str
    minimum: float
    maximum: float
    max_velocity: float


def _finite_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be a finite number")
    return number


def load_axis_limits(config_file):
    """Read the same joint-space limits used by the six-axis hardware bringup."""
    with Path(config_file).expanduser().open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not isinstance(config.get("joints"), list):
        raise ValueError("Configuration must contain a joints list")
    joints = config["joints"]
    if len(joints) != len(JOINT_NAMES):
        raise ValueError("Configuration must contain exactly six joints")

    axes = []
    for index, (joint, expected_name) in enumerate(zip(joints, JOINT_NAMES)):
        if not isinstance(joint, dict) or joint.get("joint_name") != expected_name:
            raise ValueError(f"joints[{index}] must describe {expected_name}")
        minimum = _finite_number(joint.get("min_position_rad"), f"{expected_name}.min_position_rad")
        maximum = _finite_number(joint.get("max_position_rad"), f"{expected_name}.max_position_rad")
        velocity = _finite_number(joint.get("max_velocity_rad_s"), f"{expected_name}.max_velocity_rad_s")
        if minimum >= maximum or velocity <= 0.0:
            raise ValueError(f"{expected_name} has invalid position or velocity limits")
        axes.append(AxisLimits(expected_name, minimum, maximum, velocity))
    return tuple(axes)


def slider_to_angle(axis, slider_value):
    """Map the integer slider range onto a joint's configured radian limits."""
    return axis.minimum + (axis.maximum - axis.minimum) * slider_value / SLIDER_STEPS


def angle_to_slider(axis, angle):
    """Keep the GUI handle inside its limits without changing the exact stored target."""
    fraction = (angle - axis.minimum) / (axis.maximum - axis.minimum)
    return max(0, min(SLIDER_STEPS, round(fraction * SLIDER_STEPS)))


class SliderControlModel:
    """Gate all commands on live hardware, controller, and measured joint feedback."""

    def __init__(self, axes):
        self.axes = tuple(axes)
        self.names = tuple(axis.name for axis in axes)
        self.speed = min(0.1, max(axis.max_velocity for axis in axes))
        self.actual = None
        self.target = None
        self.feedback_at = None
        self.hardware_active = False
        self.hardware_at = None
        self.controller_active = False
        self.controller_at = None
        self.ready = False
        self.dirty = False
        self.user_target_seen = False
        self.last_sent_at = None

    def observe_hardware(self, components, now):
        matches = [item for item in components if item.name == "DamiaoArm"]
        self.hardware_active = (
            len(matches) == 1 and matches[0].state.id == State.PRIMARY_STATE_ACTIVE
        )
        self.hardware_at = now

    def observe_controllers(self, controllers, now):
        matches = [item for item in controllers if item.name == "arm_controller"]
        self.controller_active = len(matches) == 1 and matches[0].state == "active"
        self.controller_at = now

    def observe_feedback(self, message, now):
        # Accept a full measured controller state, never a GUI-published /joint_states value.
        names = tuple(message.joint_names)
        positions = tuple(message.actual.positions)
        if len(names) != len(self.axes) or len(set(names)) != len(names) or len(positions) != len(names):
            self.actual = None
            self.feedback_at = None
            return
        values = dict(zip(names, positions))
        if set(values) != set(self.names):
            self.actual = None
            self.feedback_at = None
            return
        actual = tuple(values[axis.name] for axis in self.axes)
        if any(
            not math.isfinite(value) or value < axis.minimum or value > axis.maximum
            for axis, value in zip(self.axes, actual)
        ):
            self.actual = None
            self.feedback_at = None
            return
        self.actual = actual
        self.feedback_at = now

    def status(self, now):
        if not self.hardware_active or self.hardware_at is None or now - self.hardware_at > STATUS_TIMEOUT_SECONDS:
            return "等待 DamiaoArm active"
        if not self.controller_active or self.controller_at is None or now - self.controller_at > STATUS_TIMEOUT_SECONDS:
            return "等待 arm_controller active"
        if self.actual is None or self.feedback_at is None or now - self.feedback_at > STATUS_TIMEOUT_SECONDS:
            return "等待六轴新鲜实测反馈"
        return "可操作"

    def refresh(self, now):
        """On reconnect, rebase every target to feedback and discard old slider commands."""
        available = self.status(now) == "可操作"
        if available == self.ready:
            return False
        self.ready = available
        self.dirty = False
        self.user_target_seen = False
        self.last_sent_at = None
        self.target = list(self.actual) if available else None
        return True

    def set_target(self, index, angle):
        if not self.ready or self.target is None:
            return
        axis = self.axes[index]
        if not math.isfinite(angle) or angle < axis.minimum or angle > axis.maximum:
            raise ValueError(f"Target for {axis.name} is outside configured limits")
        self.target[index] = angle
        self.user_target_seen = True
        self.dirty = True

    def set_zero_targets(self):
        """Set one complete, exact zero target only when every axis can reach zero."""
        if not self.ready or any(axis.minimum > 0.0 or axis.maximum < 0.0 for axis in self.axes):
            return False
        self.target = [0.0] * len(self.axes)
        self.user_target_seen = True
        self.dirty = True
        return True

    def set_speed(self, speed):
        maximum = max(axis.max_velocity for axis in self.axes)
        if not math.isfinite(speed) or speed <= 0.0 or speed > maximum:
            raise ValueError("Test speed is outside configured velocity limits")
        self.speed = speed
        if self.ready and self.user_target_seen:
            self.dirty = True

    def finish_drag(self):
        if self.ready and self.user_target_seen:
            self.dirty = True

    def next_command(self, now):
        """Coalesce drag events and make one bounded six-axis trajectory at most every 100 ms."""
        if not self.ready or not self.dirty or self.actual is None or self.target is None:
            return None
        if self.last_sent_at is not None and now - self.last_sent_at < SEND_PERIOD_SECONDS:
            return None
        duration = max(
            MIN_TRAJECTORY_SECONDS,
            TRAJECTORY_TIME_MARGIN * max(
                abs(target - actual) / min(self.speed, axis.max_velocity)
                for axis, actual, target in zip(self.axes, self.actual, self.target)
            ),
        )
        self.dirty = False
        self.last_sent_at = now
        return tuple(self.target), duration


def make_trajectory(names, positions, duration):
    """Publish a complete position-only goal with an immediate start time."""
    message = JointTrajectory()
    message.joint_names = list(names)
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    nanoseconds = round(duration * 1_000_000_000)
    point.time_from_start.sec = nanoseconds // 1_000_000_000
    point.time_from_start.nanosec = nanoseconds % 1_000_000_000
    message.points = [point]
    return message


class SliderRosNode(Node):
    """Bridge controller-manager state and the trajectory topic without moving on startup."""

    def __init__(self, model):
        super().__init__("damiao_slider_test_gui")
        self.model = model
        self.publisher = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.create_subscription(
            JointTrajectoryControllerState,
            "/arm_controller/controller_state",
            self._on_feedback,
            10,
        )
        self.hardware_client = self.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components"
        )
        self.controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.hardware_request = None
        self.controller_request = None
        self.create_timer(0.2, self._poll_states)

    def _on_feedback(self, message):
        self.model.observe_feedback(message, time.monotonic())

    def _poll_states(self):
        # Keep one asynchronous request per service so the Qt event loop never waits for ROS.
        if self.hardware_request is None and self.hardware_client.service_is_ready():
            self.hardware_request = self.hardware_client.call_async(ListHardwareComponents.Request())
            self.hardware_request.add_done_callback(self._on_hardware_response)
        if self.controller_request is None and self.controller_client.service_is_ready():
            self.controller_request = self.controller_client.call_async(ListControllers.Request())
            self.controller_request.add_done_callback(self._on_controller_response)

    def _on_hardware_response(self, future):
        self.hardware_request = None
        try:
            response = future.result()
            self.model.observe_hardware(response.component, time.monotonic())
        except Exception as error:
            self.model.observe_hardware([], time.monotonic())
            self.get_logger().warning(f"Hardware state query failed: {error}")

    def _on_controller_response(self, future):
        self.controller_request = None
        try:
            response = future.result()
            self.model.observe_controllers(response.controller, time.monotonic())
        except Exception as error:
            self.model.observe_controllers([], time.monotonic())
            self.get_logger().warning(f"Controller state query failed: {error}")


class SliderWindow(QMainWindow):
    """Show one target slider and live measured value for each configured joint."""

    def __init__(self, model, node, executor):
        super().__init__()
        self.model = model
        self.node = node
        self.executor = executor
        self.sliders = []
        self.actual_labels = []
        self.target_labels = []
        self.setWindowTitle("Damiao 六轴滑块测试")
        self.resize(760, 360)

        content = QWidget()
        layout = QVBoxLayout(content)
        self.status_label = QLabel("等待硬件和控制器")
        layout.addWidget(self.status_label)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("速度上限 (rad/s)"))
        self.speed_box = QDoubleSpinBox()
        self.speed_box.setDecimals(6)
        self.speed_box.setSingleStep(0.01)
        highest_speed = max(axis.max_velocity for axis in model.axes)
        self.speed_box.setRange(min(0.000001, highest_speed), highest_speed)
        self.speed_box.setValue(model.speed)
        self.speed_box.valueChanged.connect(self._on_speed_changed)
        speed_row.addWidget(self.speed_box)
        speed_row.addStretch()
        self.zero_button = QPushButton("回零")
        self.zero_in_limits = all(axis.minimum <= 0.0 <= axis.maximum for axis in model.axes)
        self.zero_button.setEnabled(False)
        if not self.zero_in_limits:
            self.zero_button.setToolTip("配置中至少有一个关节的限位不包含 0 rad")
        self.zero_button.clicked.connect(self._on_zero_clicked)
        speed_row.addWidget(self.zero_button)
        layout.addLayout(speed_row)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        rows = QWidget()
        grid = QGridLayout(rows)
        # Keep the header and six axes together at the top of the scroll area.
        grid.setAlignment(Qt.AlignTop)
        grid.setVerticalSpacing(8)
        grid.setColumnStretch(2, 1)
        grid.addWidget(QLabel("关节"), 0, 0)
        grid.addWidget(QLabel("下限"), 0, 1)
        grid.addWidget(QLabel("目标滑块"), 0, 2)
        grid.addWidget(QLabel("上限"), 0, 3)
        grid.addWidget(QLabel("实测 rad"), 0, 4)
        grid.addWidget(QLabel("目标 rad"), 0, 5)
        for index, axis in enumerate(model.axes):
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, SLIDER_STEPS)
            slider.setEnabled(False)
            slider.valueChanged.connect(
                lambda value, joint_index=index: self._on_slider_changed(joint_index, value)
            )
            slider.sliderReleased.connect(self.model.finish_drag)
            actual_label = QLabel("--")
            target_label = QLabel("--")
            grid.addWidget(QLabel(axis.name), index + 1, 0)
            grid.addWidget(QLabel(f"{axis.minimum:.3f}"), index + 1, 1)
            grid.addWidget(slider, index + 1, 2)
            grid.addWidget(QLabel(f"{axis.maximum:.3f}"), index + 1, 3)
            grid.addWidget(actual_label, index + 1, 4)
            grid.addWidget(target_label, index + 1, 5)
            self.sliders.append(slider)
            self.actual_labels.append(actual_label)
            self.target_labels.append(target_label)
        scroll.setWidget(rows)
        layout.addWidget(scroll)
        self.setCentralWidget(content)

        # Drain ready ROS callbacks on the GUI thread; all widget and model updates stay single-threaded.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(20)

    def _on_speed_changed(self, value):
        self.model.set_speed(value)

    def _on_slider_changed(self, index, value):
        if not self.model.ready:
            return
        angle = slider_to_angle(self.model.axes[index], value)
        self.model.set_target(index, angle)
        self.target_labels[index].setText(f"{angle:.4f}")

    def _on_zero_clicked(self):
        # Refresh the gate before queuing one complete six-axis zero trajectory.
        if self.model.refresh(time.monotonic()):
            self._sync_sliders()
        if self.model.set_zero_targets():
            self._sync_sliders()

    def _sync_sliders(self):
        self.zero_button.setEnabled(self.model.ready and self.zero_in_limits)
        if not self.model.ready:
            for slider, label in zip(self.sliders, self.target_labels):
                slider.setEnabled(False)
                label.setText("--")
            return
        for index, (slider, axis, target) in enumerate(
            zip(self.sliders, self.model.axes, self.model.target)
        ):
            with QSignalBlocker(slider):
                slider.setValue(angle_to_slider(axis, target))
            slider.setEnabled(True)
            self.target_labels[index].setText(f"{target:.4f}")

    def _tick(self):
        for _ in range(12):
            self.executor.spin_once(timeout_sec=0.0)

        now = time.monotonic()
        if self.model.refresh(now):
            self._sync_sliders()
        self.status_label.setText(self.model.status(now))
        for label, value in zip(self.actual_labels, self.model.actual or (None,) * len(self.model.axes)):
            label.setText("--" if value is None else f"{value:.4f}")

        command = self.model.next_command(now)
        if command is not None:
            positions, duration = command
            self.node.publisher.publish(make_trajectory(self.model.names, positions, duration))

    def closeEvent(self, event):
        # Closing the GUI leaves the controller's last accepted trajectory in place.
        self.timer.stop()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description="Six-axis real-motor slider test GUI")
    parser.add_argument("--config-file", required=True, help="The YAML file used by six-axis bringup")
    arguments = parser.parse_args(remove_ros_args(sys.argv)[1:])
    axes = load_axis_limits(arguments.config_file)

    rclpy.init(args=sys.argv)
    app = QApplication([sys.argv[0]])
    model = SliderControlModel(axes)
    node = SliderRosNode(model)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    window = SliderWindow(model, node, executor)
    window.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    try:
        return app.exec_()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
