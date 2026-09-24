"""Check that the guard latches activation and rejects later hardware loss."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from lifecycle_msgs.msg import State


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "hardware_state_guard.py"
SPEC = importlib.util.spec_from_file_location("hardware_state_guard", MODULE_PATH)
GUARD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GUARD)


def component(state_id, label):
    return SimpleNamespace(
        name="DamiaoArm",
        state=SimpleNamespace(id=state_id, label=label),
    )


def test_inactive_start_then_active_is_allowed():
    tracker = GUARD.HardwareStateTracker("DamiaoArm")
    tracker.observe([component(State.PRIMARY_STATE_INACTIVE, "inactive")])
    assert not tracker.active_seen
    tracker.observe([component(State.PRIMARY_STATE_ACTIVE, "active")])
    assert tracker.active_seen


def test_fault_after_activation_stops_bringup():
    tracker = GUARD.HardwareStateTracker("DamiaoArm")
    tracker.observe([component(State.PRIMARY_STATE_ACTIVE, "active")])
    with pytest.raises(RuntimeError, match="left active state"):
        tracker.observe([component(State.PRIMARY_STATE_UNCONFIGURED, "unconfigured")])


def test_missing_hardware_is_rejected():
    tracker = GUARD.HardwareStateTracker("DamiaoArm")
    with pytest.raises(RuntimeError, match="exactly one"):
        tracker.observe([])


def test_guard_exits_on_live_service_state_transition(monkeypatch):
    # 用同进程的假 controller_manager 服务验证异步轮询后的故障退出。
    import os
    import time

    import rclpy
    from controller_manager_msgs.msg import HardwareComponentState
    from controller_manager_msgs.srv import ListHardwareComponents
    from rclpy.executors import SingleThreadedExecutor

    monkeypatch.setenv("ROS_DOMAIN_ID", str(150 + os.getpid() % 50))
    rclpy.init()
    manager = rclpy.create_node("fake_controller_manager")
    states = {"id": State.PRIMARY_STATE_INACTIVE}

    def respond(_, response):
        hardware = HardwareComponentState()
        hardware.name = "DamiaoArm"
        hardware.state.id = states["id"]
        hardware.state.label = {
            State.PRIMARY_STATE_INACTIVE: "inactive",
            State.PRIMARY_STATE_ACTIVE: "active",
            State.PRIMARY_STATE_UNCONFIGURED: "unconfigured",
        }[states["id"]]
        response.component = [hardware]
        return response

    service = manager.create_service(
        ListHardwareComponents,
        "/controller_manager/list_hardware_components",
        respond,
    )
    guard = GUARD.HardwareStateGuard("DamiaoArm")
    executor = SingleThreadedExecutor()
    executor.add_node(manager)
    executor.add_node(guard)
    try:
        # 启动阶段允许 inactive，随后观察到 active 才武装故障检测。
        first_response_at = guard._last_response_at
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and guard._last_response_at == first_response_at:
            executor.spin_once(timeout_sec=0.05)
        assert guard._last_response_at != first_response_at
        assert not guard._tracker.active_seen

        states["id"] = State.PRIMARY_STATE_ACTIVE
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not guard._tracker.active_seen:
            executor.spin_once(timeout_sec=0.05)
        assert guard._tracker.active_seen

        states["id"] = State.PRIMARY_STATE_UNCONFIGURED
        deadline = time.monotonic() + 3.0
        with pytest.raises(RuntimeError, match="left active state"):
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            pytest.fail("Guard did not stop after hardware became unconfigured")
    finally:
        executor.remove_node(guard)
        executor.remove_node(manager)
        guard.destroy_node()
        manager.destroy_service(service)
        manager.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
