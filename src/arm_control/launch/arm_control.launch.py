import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 使用 ZayV2 的机器人模型和规划配置。
    moveit_config = MoveItConfigsBuilder("zayv2_description", package_name="zayv2_moveit_config").to_moveit_configs()

    moveit_config_share = get_package_share_directory("zayv2_moveit_config")
    arm_control_share = get_package_share_directory("arm_control")
    arm_control_params = os.path.join(arm_control_share, "config", "arm_control.yaml")
    ros2_controllers_path = os.path.join(moveit_config_share, "config", "ros2_controllers.yaml")

    # 发布机器人关节状态与 TF
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "rsp.launch.py")
        )
    )

    # ros2_control 节点：管理硬件接口与控制器
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
    )

    # 加载轨迹控制器与关节状态广播器
    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "spawn_controllers.launch.py")
        )
    )

    # 启动 MoveIt move_group 规划节点
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "move_group.launch.py")
        )
    )

    # 机械臂控制节点：需加载 MoveIt 机器人模型参数，且 arm_control.yaml 放在最后避免覆盖 URDF
    arm_driver_node = Node(
        package="arm_control",
        executable="arm_driver_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            arm_control_params,
        ],
    )

    # 等待 move_group 与控制器就绪后再启动控制节点
    delayed_arm_driver = TimerAction(
        period=3.0,
        actions=[arm_driver_node],
    )

    return LaunchDescription([
        rsp_launch,
        ros2_control_node,
        spawn_controllers_launch,
        move_group_launch,
        delayed_arm_driver,
    ])
