"""Load the six-axis ZayV2 model and Damiao hardware with motion inactive."""

import math
import socket
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BUS_FIELDS = (
    "can_interface", "feedback_timeout_ms", "management_timeout_ms",
    "management_quiet_period_ms", "inactive_poll_period_ms", "max_control_period_ms",
)
JOINT_FIELDS = (
    "joint_name", "motor_name", "model", "esc_id", "mst_id", "direction",
    "zero_offset_motor_output_rad", "extra_reduction", "min_position_rad",
    "max_position_rad", "max_velocity_rad_s", "activation_position_tolerance_rad",
)
EXPECTED_JOINTS = tuple(f"joint{number}" for number in range(1, 7))


def _required(mapping, fields, label):
    if not isinstance(mapping, dict):
        raise ValueError(f"{label} must be a YAML mapping")
    missing = [field for field in fields if mapping.get(field) is None or mapping.get(field) == ""]
    if missing:
        raise ValueError(f"Fill required {label} fields: {', '.join(missing)}")


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return float(value)


def _positive_ms(bus, name):
    value = bus[name]
    if type(value) is not int or not 1 <= value <= 1000:
        raise ValueError(f"bus.{name} must be an integer from 1 to 1000 ms")
    return value


def _validate_and_describe(config, model_file, check_can=True):
    # 配置错误在创建 ROS 节点前拒绝；设备型号/零位仍须由台架记录证明。
    if not isinstance(config, dict):
        raise ValueError("Six-axis configuration must be a YAML mapping")
    bus = config.get("bus")
    joints = config.get("joints")
    _required(bus, BUS_FIELDS, "bus")
    if not isinstance(joints, list) or len(joints) != 6:
        raise ValueError("joints must contain exactly six axes")
    if type(bus.get("allow_enable_on_activate", False)) is not bool:
        raise ValueError("bus.allow_enable_on_activate must be a YAML boolean")
    interface = bus["can_interface"]
    if not isinstance(interface, str):
        raise ValueError("bus.can_interface must be a Linux network interface name")
    if check_can:
        try:
            socket.if_nametoindex(interface)
        except OSError as error:
            raise ValueError(f"CAN interface is unavailable: {interface}") from error
        if (Path("/sys/class/net") / interface / "type").read_text(encoding="ascii").strip() != "280":
            raise ValueError(f"Not a SocketCAN interface: {interface}")
    feedback_ms = _positive_ms(bus, "feedback_timeout_ms")
    management_ms = _positive_ms(bus, "management_timeout_ms")
    quiet_ms = _positive_ms(bus, "management_quiet_period_ms")
    poll_ms = _positive_ms(bus, "inactive_poll_period_ms")
    _positive_ms(bus, "max_control_period_ms")
    if quiet_ms >= management_ms or poll_ms + 6 * management_ms >= feedback_ms:
        raise ValueError("Feedback timeout must cover six management queries and polling interval")

    root = ET.parse(model_file).getroot()
    hardware = ET.SubElement(root, "ros2_control", {"name": "DamiaoArm", "type": "system"})
    plugin = ET.SubElement(hardware, "hardware")
    ET.SubElement(plugin, "plugin").text = "damiao_hardware/DamiaoSystemHardware"
    for name in BUS_FIELDS:
        ET.SubElement(plugin, "param", {"name": name}).text = str(bus[name])
    ET.SubElement(plugin, "param", {"name": "allow_enable_on_activate"}).text = str(
        bus.get("allow_enable_on_activate", False)).lower()

    motor_names = set()
    esc_ids = set()
    mst_ids = set()
    for index, axis in enumerate(joints):
        _required(axis, JOINT_FIELDS, f"joints[{index}]")
        name = EXPECTED_JOINTS[index]
        if axis["joint_name"] != name:
            raise ValueError(f"joints[{index}].joint_name must be {name}")
        if not isinstance(axis["motor_name"], str) or not isinstance(axis["model"], str):
            raise ValueError(f"{name} motor_name and model must be strings")
        esc = axis["esc_id"]
        mst = axis["mst_id"]
        if type(esc) is not int or type(mst) is not int or not 1 <= esc <= 15 or not 1 <= mst <= 2047:
            raise ValueError(f"{name} has an invalid ESC_ID or MST_ID")
        if axis["motor_name"] in motor_names or esc in esc_ids or mst in mst_ids:
            raise ValueError(f"{name} duplicates a motor name or CAN ID")
        motor_names.add(axis["motor_name"])
        esc_ids.add(esc)
        mst_ids.add(mst)
        if type(axis["direction"]) is not int or axis["direction"] not in (-1, 1):
            raise ValueError(f"{name}.direction must be -1 or 1")
        for field in ("zero_offset_motor_output_rad", "extra_reduction", "min_position_rad",
                      "max_position_rad", "max_velocity_rad_s", "activation_position_tolerance_rad"):
            _finite(axis[field], f"{name}.{field}")
        minimum = float(axis["min_position_rad"])
        maximum = float(axis["max_position_rad"])
        speed = float(axis["max_velocity_rad_s"])
        if minimum >= maximum or axis["extra_reduction"] <= 0 or speed <= 0 \
                or axis["activation_position_tolerance_rad"] <= 0:
            raise ValueError(f"{name} has invalid limits or reduction")
        model_joint = root.find(f"./joint[@name='{name}']")
        if model_joint is None or model_joint.find("limit") is None:
            raise ValueError(f"Robot model is missing {name} limits")
        model_limit = model_joint.find("limit")
        if minimum < float(model_limit.get("lower")) or maximum > float(model_limit.get("upper")) \
                or speed > float(model_limit.get("velocity")):
            raise ValueError(f"{name} limits exceed the current ZayV2 model")
        control_joint = ET.SubElement(hardware, "joint", {"name": name})
        for field in JOINT_FIELDS:
            if field != "joint_name":
                ET.SubElement(control_joint, "param", {"name": field}).text = str(axis[field])
        ET.SubElement(control_joint, "command_interface", {"name": "position"})
        ET.SubElement(control_joint, "state_interface", {"name": "position"})
        ET.SubElement(control_joint, "state_interface", {"name": "velocity"})
    for first_index in range(6):
        for second_index in range(first_index + 1, 6):
            first = joints[first_index]
            second = joints[second_index]
            if (first["mst_id"] & 0xFF) == second["esc_id"] \
                    or (second["mst_id"] & 0xFF) == first["esc_id"]:
                raise ValueError("Cross-axis feedback/command CAN ID collision")
    return ET.tostring(root, encoding="unicode")


def _start_nodes(context):
    config_file = Path(LaunchConfiguration("config_file").perform(context)).expanduser().resolve()
    with config_file.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    model_share = Path(get_package_share_directory("zayv2_description"))
    model = model_share / "urdf" / "zayv2_description.urdf"
    description = _validate_and_describe(config, model)
    share = Path(get_package_share_directory("zayv2_bringup"))
    controllers = str(share / "config" / "controllers_six_axis.yaml")
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": description}, controllers],
        output="screen",
    )
    # 控制器管理器退出时同步关闭广播器和机器人状态发布器。
    shutdown_on_control_exit = RegisterEventHandler(
        OnProcessExit(target_action=control_node,
            on_exit=[EmitEvent(event=Shutdown(reason="controller_manager exited"))]),
    )
    return [
        shutdown_on_control_exit,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description}],
            output="screen",
        ),
        control_node,
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "--inactive", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("zayv2_bringup"))
    example = share / "config" / "six_axis.example.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=str(example)),
        OpaqueFunction(function=_start_nodes),
    ])
