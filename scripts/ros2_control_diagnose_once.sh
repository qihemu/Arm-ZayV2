#!/usr/bin/env bash
# 临时诊断：在 damiao_six_axis.launch.py 已运行时使用，结果写入 logs/ 目录。
set -eo pipefail

WS_ROOT="/home/qihemu/qihemu_ws/Arm-ZayV2"
LOG_DIR="${WS_ROOT}/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/ros2_control_diagnose_$(date +%Y%m%d_%H%M%S).log"

exec > >(tee -a "${LOG_FILE}") 2>&1

section()
{
    echo ""
    echo "======================================================================"
    echo ">>> $*"
    echo ">>> $(date -Iseconds)"
    echo "======================================================================"
}

run()
{
    echo "+ $*"
    "$@"
    local code=$?
    echo "[exit ${code}] $*"
    return "${code}"
}

section "环境"
echo "LOG_FILE=${LOG_FILE}"
echo "HOST=$(hostname)"
echo "PWD=$(pwd)"
# ROS setup 会读未定义变量，source 前临时关闭 nounset
set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
else
    echo "WARN: /opt/ros/humble/setup.bash 不存在"
fi
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${WS_ROOT}/install/setup.bash"
else
    echo "WARN: ${WS_ROOT}/install/setup.bash 不存在，先 colcon build"
fi
run env | grep -E '^(ROS_DISTRO|AMENT_PREFIX_PATH=)' || true

section "launch 侧 YAML（install 与源码对比 allow_enable）"
for f in \
    "${WS_ROOT}/install/zayv2_bringup/share/zayv2_bringup/config/six_axis.example.yaml" \
    "${WS_ROOT}/src/zayv2_bringup/config/six_axis.example.yaml"
do
    if [[ -f "${f}" ]]; then
        echo "--- ${f} ---"
        grep -n "allow_enable_on_activate" "${f}" || true
    else
        echo "MISSING: ${f}"
    fi
done

section "CAN can0"
run ip -details link show can0 || true
if [[ -r /sys/class/net/can0/type ]]; then
    echo "can0 type=$(cat /sys/class/net/can0/type)"
fi

section "ROS 图（controller_manager 是否在）"
run ros2 node list || true
run ros2 service list | grep -E 'controller_manager|list_hardware|list_controllers' || true

section "初始状态（不做 lifecycle 变更）"
run ros2 control list_hardware_components || true
run ros2 control list_hardware_interfaces || true
run ros2 control list_controllers || true
run timeout 5 ros2 topic echo /joint_states --once || true

section "Lifecycle：DamiaoArm inactive（unconfigured 时也会尝试 configure）"
run ros2 control set_hardware_component_state DamiaoArm inactive || true
sleep 1
run ros2 control list_hardware_components || true
run ros2 control list_hardware_interfaces | grep -E 'joint[1-6]/position' || true

section "Lifecycle：DamiaoArm active（需 allow_enable_on_activate: true 且 CAN/电机正常）"
run ros2 control set_hardware_component_state DamiaoArm active || true
sleep 2
run ros2 control list_hardware_components || true
run ros2 control list_hardware_interfaces | grep -E 'joint[1-6]/position' || true
run timeout 5 ros2 topic echo /joint_states --once || true

section "Lifecycle：arm_controller active"
run ros2 control set_controller_state arm_controller active || true
sleep 1
run ros2 control list_controllers || true
run ros2 control list_hardware_interfaces | grep -E 'joint[1-6]/position' || true

section "Action 接口（JTC 是否起来）"
run ros2 action info /arm_controller/follow_joint_trajectory || true

section "结束状态"
run ros2 control list_hardware_components || true
run ros2 control list_controllers || true

section "完成"
echo "请把此文件全文发给调试：${LOG_FILE}"
echo "launch 终端里与本脚本时间重叠的 [ros2_control_node] / [damiao_hardware] 行也请一并复制。"
