"""
启动 MoveIt Servo 节点（官方 servo_node_main）。

前提：robot_state_publisher、ros2_control、arm_controller 已运行。

用法：
  ros2 launch zayv2_moveit_config servo.launch.py
  ros2 launch zayv2_moveit_config servo.launch.py auto_start:=false
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_servo_params(package_name: str, file_path: str) -> dict:
    """从 yaml 加载 Servo 参数并包装为 moveit_servo 命名空间。"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        servo_yaml = yaml.safe_load(file)
    return {"moveit_servo": servo_yaml}


def generate_launch_description():
    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="true",
        description="启动后自动调用 /servo_node/start_servo",
    )

    moveit_config = MoveItConfigsBuilder("zayv2_description", package_name="zayv2_moveit_config").to_moveit_configs()
    servo_params = load_servo_params("zayv2_moveit_config", "config/servo.yaml")

    # 官方 Servo 节点
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.joint_limits,
            moveit_config.robot_description_kinematics,
        ],
    )

    # 等待控制器与 PSM 就绪后自动 start_servo
    start_servo = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("auto_start")),
        cmd=[
            "ros2",
            "service",
            "call",
            "/servo_node/start_servo",
            "std_srvs/srv/Trigger",
            "{}",
        ],
        output="screen",
    )
    delayed_start_servo = TimerAction(period=5.0, actions=[start_servo])

    return LaunchDescription([
        auto_start_arg,
        servo_node,
        delayed_start_servo,
    ])
