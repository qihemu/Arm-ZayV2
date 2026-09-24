"""Software-only checks for the six-axis slider command gate and Qt presentation."""

import importlib.util
import math
import os
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QLabel
from lifecycle_msgs.msg import State


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "damiao_slider_test_gui.py"
SPEC = importlib.util.spec_from_file_location("damiao_slider_test_gui", MODULE_PATH)
GUI = importlib.util.module_from_spec(SPEC)
import sys
sys.modules[SPEC.name] = GUI
SPEC.loader.exec_module(GUI)


def axes():
    return tuple(
        GUI.AxisLimits(f"joint{index}", -1.0, 1.0, 0.5 + index * 0.1)
        for index in range(1, 7)
    )


def feedback(positions, names=None):
    return SimpleNamespace(
        joint_names=list(names or GUI.JOINT_NAMES),
        actual=SimpleNamespace(positions=list(positions)),
    )


def activate(model, now=10.0, positions=None, refresh=True):
    model.observe_hardware(
        [SimpleNamespace(name="DamiaoArm", state=SimpleNamespace(id=State.PRIMARY_STATE_ACTIVE))],
        now,
    )
    model.observe_controllers(
        [SimpleNamespace(name="arm_controller", state="active")],
        now,
    )
    model.observe_feedback(feedback(positions or [0.0] * 6), now)
    if refresh:
        assert model.refresh(now)


def test_loads_six_joint_limits_and_rejects_invalid_config(tmp_path):
    config = tmp_path / "six_axis.yaml"
    lines = ["joints:"]
    for index in range(1, 7):
        lines.extend([
            f"  - joint_name: joint{index}",
            "    min_position_rad: -1.0",
            "    max_position_rad: 1.0",
            "    max_velocity_rad_s: 0.5",
        ])
    config.write_text("\n".join(lines), encoding="utf-8")
    loaded = GUI.load_axis_limits(config)
    assert len(loaded) == 6
    assert loaded[0].name == "joint1"
    assert loaded[-1].max_velocity == 0.5

    config.write_text("\n".join(lines).replace("joint3", "joint2"), encoding="utf-8")
    with pytest.raises(ValueError, match="joint3"):
        GUI.load_axis_limits(config)
    config.write_text("\n".join(lines).replace("max_position_rad: 1.0", "max_position_rad: .nan", 1), encoding="utf-8")
    with pytest.raises(ValueError, match="finite"):
        GUI.load_axis_limits(config)


def test_slider_mapping_and_initial_target_preserve_exact_feedback():
    model = GUI.SliderControlModel(axes())
    measured = [0.123456, -0.456789, 0.2, 0.3, -0.1, 0.0]
    activate(model, positions=measured)
    assert model.target == measured
    assert model.next_command(10.0) is None

    for axis, value in zip(model.axes, measured):
        slider_value = GUI.angle_to_slider(axis, value)
        assert 0 <= slider_value <= GUI.SLIDER_STEPS
        assert abs(GUI.slider_to_angle(axis, slider_value) - value) <= 0.00011


def test_multiaxis_goals_are_coalesced_and_bounded_by_selected_speed():
    model = GUI.SliderControlModel(axes())
    activate(model)
    model.set_speed(0.1)
    model.set_target(0, 0.2)
    model.set_target(2, -0.1)
    first = model.next_command(10.0)
    assert first is not None
    assert first[0] == pytest.approx((0.2, 0.0, -0.1, 0.0, 0.0, 0.0))
    assert first[1] == pytest.approx(2.4)

    model.set_target(0, 0.3)
    model.finish_drag()
    assert model.next_command(10.05) is None
    second = model.next_command(10.101)
    assert second is not None
    assert second[0][0] == pytest.approx(0.3)
    assert second[1] == pytest.approx(3.6)
    assert model.next_command(10.2) is None

    message = GUI.make_trajectory(model.names, *second)
    assert message.joint_names == list(GUI.JOINT_NAMES)
    assert len(message.points) == 1
    assert message.points[0].positions[0] == pytest.approx(0.3)
    assert message.points[0].time_from_start.sec == 3
    assert message.points[0].time_from_start.nanosec == 600_000_000


def test_each_axis_uses_its_configured_velocity_cap():
    model = GUI.SliderControlModel(axes())
    activate(model)
    model.set_speed(max(axis.max_velocity for axis in model.axes))
    model.set_target(0, 0.6)
    result = model.next_command(10.0)
    assert result[1] == pytest.approx(1.2)


def test_invalid_or_stale_feedback_disables_and_discards_old_target():
    model = GUI.SliderControlModel(axes())
    activate(model)
    model.set_target(0, 0.4)
    assert model.refresh(10.51)
    assert not model.ready
    assert model.target is None
    assert model.next_command(10.51) is None

    model.observe_hardware(
        [SimpleNamespace(name="DamiaoArm", state=SimpleNamespace(id=State.PRIMARY_STATE_ACTIVE))],
        11.0,
    )
    model.observe_controllers([SimpleNamespace(name="arm_controller", state="active")], 11.0)
    model.observe_feedback(feedback([0.1] * 6), 11.0)
    assert model.refresh(11.0)
    assert model.target == [0.1] * 6
    assert model.next_command(11.0) is None

    model.observe_feedback(feedback([math.nan] + [0.0] * 5), 11.1)
    assert model.refresh(11.1)
    assert not model.ready
    model.observe_feedback(feedback([0.0] * 5, names=GUI.JOINT_NAMES[:-1]), 11.2)
    assert model.next_command(11.2) is None


def test_hardware_or_controller_inactive_blocks_pending_motion():
    model = GUI.SliderControlModel(axes())
    activate(model)
    model.set_target(0, 0.5)
    model.observe_hardware(
        [SimpleNamespace(name="DamiaoArm", state=SimpleNamespace(id=State.PRIMARY_STATE_INACTIVE))],
        10.01,
    )
    assert model.refresh(10.01)
    assert not model.ready
    assert model.next_command(10.01) is None

    activate(model, now=11.0)
    model.set_target(1, 0.2)
    model.observe_controllers([SimpleNamespace(name="arm_controller", state="inactive")], 11.01)
    assert model.refresh(11.01)
    assert not model.ready
    assert model.next_command(11.01) is None


def test_short_move_has_minimum_trajectory_duration():
    model = GUI.SliderControlModel(axes())
    activate(model)
    model.set_target(0, 0.001)
    result = model.next_command(10.0)
    assert result[1] == pytest.approx(GUI.MIN_TRAJECTORY_SECONDS)


def test_zero_target_is_complete_and_respects_joint_limits():
    model = GUI.SliderControlModel(axes())
    assert not model.set_zero_targets()
    activate(model, positions=[0.3] * 6)
    assert model.set_zero_targets()
    assert model.target == [0.0] * 6
    assert model.next_command(10.0)[0] == (0.0,) * 6

    limited_axes = (GUI.AxisLimits("joint1", 0.1, 1.0, 0.5),) + axes()[1:]
    limited_model = GUI.SliderControlModel(limited_axes)
    activate(limited_model, positions=[0.3] * 6)
    assert not limited_model.set_zero_targets()
    assert limited_model.next_command(10.0) is None


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self):
        self.publisher = FakePublisher()


class FakeExecutor:
    def spin_once(self, timeout_sec):
        assert timeout_sec == 0.0


def test_offscreen_gui_starts_without_motion_and_sends_only_after_slider_change():
    app = QApplication.instance() or QApplication([])
    model = GUI.SliderControlModel(axes())
    node = FakeNode()
    window = GUI.SliderWindow(model, node, FakeExecutor())
    window.timer.stop()
    window.show()
    app.processEvents()
    try:
        # Header and slider rows should remain a compact group when the window expands.
        header = next(label for label in window.findChildren(QLabel) if label.text() == "关节")
        header_y = header.mapTo(window, header.rect().topLeft()).y()
        first_y = window.sliders[0].mapTo(window, window.sliders[0].rect().topLeft()).y()
        second_y = window.sliders[1].mapTo(window, window.sliders[1].rect().topLeft()).y()
        assert first_y - header_y < 2 * (second_y - first_y)
        assert all(not slider.isEnabled() for slider in window.sliders)
        activate(model, now=0.0, positions=[0.123456] * 6, refresh=False)
        # Use the current monotonic clock for the GUI's tick and do not move on rebase.
        current = GUI.time.monotonic()
        model.hardware_at = current
        model.controller_at = current
        model.feedback_at = current
        window._tick()
        assert all(slider.isEnabled() for slider in window.sliders)
        assert node.publisher.messages == []
        assert model.target == [0.123456] * 6

        window.sliders[1].setValue(window.sliders[1].value() + 100)
        window._tick()
        assert len(node.publisher.messages) == 1
        assert len(node.publisher.messages[0].points[0].positions) == 6
    finally:
        window.close()
        app.processEvents()


def test_offscreen_zero_button_sends_one_complete_goal_and_disables_on_loss():
    app = QApplication.instance() or QApplication([])
    model = GUI.SliderControlModel(axes())
    node = FakeNode()
    window = GUI.SliderWindow(model, node, FakeExecutor())
    window.timer.stop()
    window.show()
    app.processEvents()
    try:
        assert not window.zero_button.isEnabled()
        activate(model, now=0.0, positions=[0.3] * 6, refresh=False)
        current = GUI.time.monotonic()
        model.hardware_at = current
        model.controller_at = current
        model.feedback_at = current
        window._tick()
        assert window.zero_button.isEnabled()
        assert node.publisher.messages == []

        window.zero_button.click()
        assert model.target == [0.0] * 6
        assert all(
            slider.value() == GUI.angle_to_slider(axis, 0.0)
            for slider, axis in zip(window.sliders, model.axes)
        )
        assert node.publisher.messages == []
        window._tick()
        assert len(node.publisher.messages) == 1
        assert list(node.publisher.messages[0].points[0].positions) == [0.0] * 6

        model.observe_controllers([SimpleNamespace(name="arm_controller", state="inactive")], current)
        window._tick()
        assert not window.zero_button.isEnabled()
    finally:
        window.close()
        app.processEvents()
