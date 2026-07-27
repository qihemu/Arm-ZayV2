"""
MoveIt Servo 键盘遥操作 launch 文件

须在可交互 TTY 终端中运行（launch 子进程 stdin 非 TTY 时会自动使用 /dev/tty）。

用法：
  # 终端 1：启动 Servo 栈
  ros2 launch arm_control servo_demo.launch.py

  # 终端 2：键盘遥操作
  ros2 launch arm_control servo_keyboard.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    arm_control_share = get_package_share_directory("arm_control")
    servo_keyboard_yaml = os.path.join(arm_control_share, "config", "servo_keyboard.yaml")

    servo_keyboard_node = Node(
        package="arm_control",
        executable="servo_keyboard_node",
        output="screen",
        parameters=[servo_keyboard_yaml],
    )

    return LaunchDescription([servo_keyboard_node])
