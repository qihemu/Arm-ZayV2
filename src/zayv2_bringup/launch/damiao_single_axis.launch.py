"""Start one Damiao axis with hardware and trajectory controller inactive."""

from pathlib import Path
import socket

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


REQUIRED_KEYS = (
    "joint_name", "motor_name", "model", "can_interface", "esc_id", "mst_id",
    "direction", "zero_offset_motor_output_rad", "extra_reduction",
    "min_position_rad", "max_position_rad", "max_velocity_rad_s",
    "activation_position_tolerance_rad", "feedback_timeout_ms",
    "management_timeout_ms", "management_quiet_period_ms",
    "inactive_poll_period_ms", "max_control_period_ms",
)


def _start_nodes(context):
    # 在创建 ROS 节点之前检查模板占位值和控制器固定的关节名。
    config_file = Path(LaunchConfiguration("config_file").perform(context)).expanduser().resolve()
    with config_file.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"Expected a YAML mapping: {config_file}")
    missing = [key for key in REQUIRED_KEYS if config.get(key) is None or config.get(key) == ""]
    if missing:
        raise ValueError(f"Fill required single-axis configuration: {', '.join(missing)}")
    if config["joint_name"] != "joint1":
        raise ValueError("DEV-07 controller configuration requires joint_name: joint1")
    if type(config.get("allow_enable_on_activate", False)) is not bool:
        raise ValueError("allow_enable_on_activate must be a YAML boolean")
    if not isinstance(config["can_interface"], str):
        raise ValueError("can_interface must be a Linux network interface name")
    try:
        socket.if_nametoindex(config["can_interface"])
    except OSError as error:
        raise ValueError(f"CAN interface is unavailable: {config['can_interface']}") from error
    # Linux ARPHRD_CAN 为 280；避免把普通网卡误当作 SocketCAN。
    interface_type = Path("/sys/class/net") / config["can_interface"] / "type"
    if interface_type.read_text(encoding="ascii").strip() != "280":
        raise ValueError(f"Not a SocketCAN interface: {config['can_interface']}")

    share = Path(get_package_share_directory("zayv2_bringup"))
    description = xacro.process_file(
        str(share / "urdf" / "damiao_single_axis.urdf.xacro"),
        mappings={"config_file": str(config_file)},
    ).toxml()
    controllers = str(share / "config" / "controllers_single_axis.yaml")
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": description}, controllers],
        output="screen",
    )
    # 控制器管理器若因配置或设备故障退出，停止其余启动进程。
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
            arguments=["single_axis_controller", "--inactive", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("zayv2_bringup"))
    example = share / "config" / "single_axis.example.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=str(example)),
        OpaqueFunction(function=_start_nodes),
    ])
