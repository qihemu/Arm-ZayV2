import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import launch_ros
import launch


def is_node_running(node_name):
    """检查指定名称的节点是否正在运行"""
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        return f'/{node_name}' in result.stdout or node_name in result.stdout
    except Exception as e:
        print(f"检查节点 {node_name} 时出错: {e}")
        return False

def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_ros_description').find('robot_ros_description')
    print(pkg_share)
    urdf_path = os.path.join(pkg_share, 'urdf/aubo_i5.urdf')
    rviz_path = os.path.join(pkg_share, 'rviz/aubo_robot.rviz')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    # 检查节点是否已经运行
    robot_state_publisher_running = is_node_running('robot_state_publisher')
    arm_driver_running = is_node_running('arm_driver')

    print(f"robot_state_publisher 运行状态: {robot_state_publisher_running}")
    print(f"arm_driver 运行状态: {arm_driver_running}")


    ld = LaunchDescription()

    # 只有在节点未运行时才启动 robot_state_publisher
    if not robot_state_publisher_running:
        print("启动 robot_state_publisher 节点")
        ld.add_action(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[rsp_params]
            )
        )
    else:
        print("robot_state_publisher 已在运行，跳过启动")

    # arm_driver 必须已在运行，否则启动失败
    if not arm_driver_running:
        raise RuntimeError("arm_driver 未运行，请先启动 arm_driver_for_aubo 节点后再运行此 launch 文件")
    print("arm_driver 已在运行")

    # 启动RViz2节点
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path],
            # parameters=[{'use_sim_time': False}]
        )
    )
    
    return ld
