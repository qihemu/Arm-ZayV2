"""
ZayV2 MoveIt Servo 完整演示：仿真栈 + Servo + 可选键盘遥操作。

用法：
  # 终端 1：启动 Servo 栈（不含键盘）
  ros2 launch arm_control servo_demo.launch.py

  # 终端 2：键盘遥操作（须在可交互 TTY 中运行）
  ros2 launch arm_control servo_keyboard.launch.py

  # 或一条命令同时启动键盘（仅当 launch 终端为 TTY 时可用）
  ros2 launch arm_control servo_demo.launch.py with_keyboard:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    with_keyboard_arg = DeclareLaunchArgument(
        "with_keyboard",
        default_value="false",
        description="在同一 launch 中启动 servo_keyboard_node（需要 TTY）",
    )

    arm_control_share = get_package_share_directory("arm_control")
    servo_initial_positions = os.path.join(arm_control_share, "config", "servo_initial_positions.yaml")

    # 仅为 Servo 仿真加载避开腕部奇异点的初始姿态。
    moveit_config = (
        MoveItConfigsBuilder("zayv2_description", package_name="zayv2_moveit_config")
        .robot_description(mappings={"initial_positions_file": servo_initial_positions})
        .to_moveit_configs()
    )
    moveit_config_share = get_package_share_directory("zayv2_moveit_config")
    ros2_controllers_path = os.path.join(moveit_config_share, "config", "ros2_controllers.yaml")
    servo_keyboard_yaml = os.path.join(arm_control_share, "config", "servo_keyboard.yaml")

    # RSP 与 ros2_control 使用同一份 URDF，确保初始关节状态一致。
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        respawn=True,
        parameters=[moveit_config.robot_description, {"publish_frequency": 15.0}],
    )

    # ros2_control 与轨迹控制器
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
    )
    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "spawn_controllers.launch.py")
        )
    )

    # move_group：提供规划场景（Servo 配置 is_primary_planning_scene_monitor: false）
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "move_group.launch.py")
        )
    )

    # 官方 Servo 节点（含 auto_start）
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "servo.launch.py")
        )
    )

    # 键盘遥操作（可选）
    servo_keyboard_node = Node(
        condition=IfCondition(LaunchConfiguration("with_keyboard")),
        package="arm_control",
        executable="servo_keyboard_node",
        output="screen",
        parameters=[servo_keyboard_yaml],
    )
    delayed_keyboard = TimerAction(period=6.0, actions=[servo_keyboard_node])

    return LaunchDescription([
        with_keyboard_arg,
        rsp_node,
        ros2_control_node,
        spawn_controllers_launch,
        move_group_launch,
        servo_launch,
        delayed_keyboard,
    ])
