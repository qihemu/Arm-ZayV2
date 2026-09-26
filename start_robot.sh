#!/usr/bin/env bash
# 前台运行一个模块，Ctrl+C交由ros2 launch正常关闭；不自动使能或写电机参数。
set -eo pipefail
robot_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
usage() {
    echo "用法: $0 [lidar|wheel|mapping|localization|amcl] [ROS launch参数:=值 ...]"
    echo '示例: start_robot.sh lidar driver:=false'
    echo '定位需map_directory:=绝对路径；图定位另需initial_x/y/yaw。'
    echo 'wheel模式取自根YAML；建图/定位另开终端启动lidar并提供有效odom TF。'
}
if [[ ${1:-} == --help || ${1:-} == -h ]]; then usage; exit 0; fi
if [[ $# == 0 ]]; then
    echo '启动: 1) 雷达感知  2) H55轮控（YAML模式）  3) 建图  4) 图定位  5) AMCL  q)退出'
    read -r -p '选择: ' selection || exit 1
    case "$selection" in
        1) set -- lidar ;; 2) set -- wheel ;; 3) set -- mapping ;;
        4|5)
            read -r -p '地图版本目录（绝对路径）: ' map_dir || exit 1
            if [[ "$selection" == 4 ]]; then
                read -r -p '初始x y yaw（米、弧度，空格分隔）: ' x y yaw || exit 1
                set -- localization "map_directory:=$map_dir" "initial_x:=$x" "initial_y:=$y" "initial_yaw:=$yaw"
            else
                set -- amcl "map_directory:=$map_dir"
            fi ;;
        q) exit 0 ;; *) echo '无效选择' >&2; exit 2 ;;
    esac
fi
target=$1
shift
# 参数以数组传递，路径可含空格；不通过eval执行输入。
case "$target" in
    wheel)
        package=robot_wheel_control
        source "$robot_root/scripts/wheel_real_bench_env.bash"
        command=(ros2 launch "$package" wheel_bench.launch.py "config_file:=$H55_BENCH_CONFIG")
        lock_name=wheel ;;
    lidar|mapping|localization|amcl)
        package=lidar_slam
        source "$robot_root/scripts/lidar_env.bash"
        if [[ "$target" == lidar ]]; then
            command=(ros2 launch "$package" c1.launch.py)
            lock_name=lidar
        else
            command=(ros2 launch "$package" slam.launch.py "mode:=$target")
            lock_name=slam
        fi ;;
    *) usage >&2; exit 2 ;;
esac
# 快速拒绝未构建入口；不启动其他包或操作系统服务。
ros2 pkg prefix "$package" >/dev/null || { echo '请先运行build_robot.sh构建对应模块' >&2; exit 1; }
log_root="${ARM_ZAY_LOG_DIR:-$robot_root/logs/runtime}"
mkdir -p "$log_root/$package"
exec 9>"$log_root/.$lock_name.lock"
flock -n 9 || { echo '该模块已有通过本脚本启动的会话' >&2; exit 1; }
session="$log_root/$package/$(date +%Y%m%d-%H%M%S)-$$-$target"
mkdir -p "$session/ros"
export ROS_LOG_DIR="$session/ros"
export RCUTILS_COLORIZED_OUTPUT=0
# 每个包及每种模式都有latest入口；历史会话不会自动删除。
touch "$session/console.log"
ln -sfn "$(basename "$session")" "$log_root/$package/latest"
ln -sfn "$(basename "$session")" "$log_root/$package/latest-$target"
# launch拥有独立进程组，终端信号只转发一次，由launch负责关闭子节点。
# 避免父子同时收到Ctrl+C导致Python节点清理阶段再次中断。
trap ':' INT TERM
python3 - "$session/console.log" "$target" "${command[@]}" "$@" <<'PYRUN'
import os
import shlex
import signal
import subprocess
import sys

log_path, target, *command = sys.argv[1:]
with open(log_path, 'a', buffering=1) as log:
    def output(text):
        log.write(text)
        print(text, end='', flush=True)

    output(f"模块={target} ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')} 日志={log_path}\n")
    output('命令: ' + shlex.join(command) + '\n')
    # 子进程输出逐行同时写入终端和日志；保留实际启动退出码。
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, errors='replace', start_new_session=True)
    stopping = False

    def shutdown(signum, frame):
        global stopping
        if not stopping and process.poll() is None:
            stopping = True
            try:
                process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    for line in process.stdout:
        output(line)
    code = process.wait()
    sys.exit(code if code >= 0 else 128 - code)
PYRUN
