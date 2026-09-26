#!/usr/bin/env bash
# 本机H55与C1模块构建入口，保留各自ASCII缓存，不安装依赖或操作硬件。
set -eo pipefail
robot_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    echo "用法: $0 [all|wheel|lidar|core]"
    echo "all = H55轮控及C1感知（含共享依赖），不包含原机械臂/MoveIt工程。"
    exit 0
fi
if [[ $# == 0 ]]; then
    echo '编译模块: 1) 轮控+雷达  2) 轮控  3) 雷达  4) 仅core  q)退出'
    read -r -p '选择: ' selection || exit 1
    case "$selection" in
        1) set -- all ;; 2) set -- wheel ;; 3) set -- lidar ;; 4) set -- core ;; q) exit 0 ;;
        *) echo '无效选择' >&2; exit 2 ;;
    esac
fi
[[ $# == 1 ]] || { echo '只接受一个编译目标' >&2; exit 2; }
case "$1" in all|wheel|lidar|core) ;; *) echo "未知编译目标: $1" >&2; exit 2 ;; esac
# 同一源码工作空间禁止并发构建，防止缓存复制和安装互相覆盖。
mkdir -p "$robot_root/logs/build"
exec 9>"$robot_root/logs/build/.lock"
flock -n 9 || { echo '已有构建正在运行' >&2; exit 1; }

# wheel模块：保留原构建依赖和旧缓存迁移。
build_wheel() (
    wheel_source_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
    wheel_build_root="${XDG_CACHE_HOME:-$HOME/.cache}/h55-wheel-control"
    wheel_deps_root="$(dirname "$wheel_source_root")/.local/wheel-ros-deps/root"
    source /opt/ros/humble/setup.bash
    if [[ -d "$wheel_deps_root/opt/ros/humble" ]]; then
        export AMENT_PREFIX_PATH="$wheel_deps_root/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
        export CMAKE_PREFIX_PATH="$wheel_deps_root/opt/ros/humble:$wheel_deps_root/usr:${CMAKE_PREFIX_PATH:-}"
        export LD_LIBRARY_PATH="$wheel_deps_root/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
        export PYTHONPATH="$wheel_deps_root/opt/ros/humble/local/lib/python3.10/dist-packages:$wheel_deps_root/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
    fi
    python3 - "$wheel_source_root" "$wheel_build_root" <<'PY'
import shutil, sys
from pathlib import Path
source, target = map(Path, sys.argv[1:])
# 本次轮控测试已在验证后移出Git仓库，清理其旧源码与CMake缓存，防止增量构建残留。
retired = target / 'src/robot_wheel_control/test'
if retired.exists():
    shutil.rmtree(retired)
    old_build = target / 'build/robot_wheel_control'
    if old_build.exists():
        shutil.rmtree(old_build)
# 清除本轮明确迁出运行包的旧缓存文件，避免增量复制留下模拟入口/公开头文件。
for relative in (
    'src/robot_wheel_control/src/mock_transport.cpp',
    'src/robot_wheel_control/include/robot_wheel_control/mock_transport.hpp',
    'src/robot_wheel_control/test/ros_smoke.py',
    'install/robot_wheel_control/include/robot_wheel_control/mock_transport.hpp',
):
    obsolete = target / relative
    if obsolete.is_file():
        obsolete.unlink()
for package in ('damiao_core', 'robot_interfaces', 'robot_wheel_control', 'damiao_hardware', 'damiao_tools'):
    shutil.copytree(source / 'src' / package, target / 'src' / package, dirs_exist_ok=True)
shutil.copytree(source / 'config', target / 'config', dirs_exist_ok=True)
PY
    cd "$wheel_build_root"
    colcon build --base-paths src --packages-select damiao_core robot_interfaces robot_wheel_control damiao_hardware damiao_tools --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
    printf 'Build complete: source %s/scripts/wheel_env.bash\n' "$wheel_source_root"
)

# lidar模块：保留原构建依赖和旧缓存迁移。
build_lidar() (
    lidar_source_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
    lidar_build_root="${XDG_CACHE_HOME:-$HOME/.cache}/arm-zay-lidar"
    source /opt/ros/humble/setup.bash
    python3 - "$lidar_source_root" "$lidar_build_root" <<'PY'
import shutil
import sys
from pathlib import Path
source, target = map(Path, sys.argv[1:])
# 清理本次迁出的旧包内配置缓存，避免同名旧配置残留误导使用者。
retired = target / 'src/lidar_slam/config'
if retired.exists():
    shutil.rmtree(retired)
for old in ('perception.yaml', 'mount.yaml', 'slam.yaml', 'amcl.yaml', 'record_qos.yaml', 'map_context.example.yaml'):
    installed = target / 'install/lidar_slam/share/lidar_slam/config' / old
    if installed.is_file():
        installed.unlink()
# 本轮统一配置后移除工具自己维护的八份旧缓存，唯一保留lidar_slam.yaml。
for old in ('lidar_driver.yaml', 'lidar_mount.yaml', 'lidar_perception.yaml', 'lidar_data.yaml',
            'lidar_amcl.yaml', 'lidar_record_qos.yaml', 'lidar_map_context.example.yaml', 'lidar_nav2.example.yaml'):
    for directory in (target / 'config', target / 'install/lidar_slam/share/lidar_slam/config'):
        obsolete = directory / old
        if obsolete.is_file():
            obsolete.unlink()
for package in ('robot_interfaces', 'lidar_slam'):
    shutil.copytree(source / 'src' / package, target / 'src' / package, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
shutil.copytree(source / 'config', target / 'config', dirs_exist_ok=True)
PY
    cd "$lidar_build_root"
    colcon build --base-paths src --packages-select robot_interfaces lidar_slam --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
    printf 'Build complete: source %s/scripts/lidar_env.bash\n' "$lidar_source_root"
)

# core可单独开发；放在独立缓存，避免与colcon构建目录混用。
build_core() (
    core_cache="${XDG_CACHE_HOME:-$HOME/.cache}/arm-zay-core"
    cmake -S "$robot_root/src/damiao_core" -B "$core_cache" -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build "$core_cache" --parallel 2
)
build_selected() {
    case "$1" in
        wheel) build_wheel ;; lidar) build_lidar ;; core) build_core ;;
        all) build_wheel; build_lidar ;;
    esac
}
# pipefail保留真实构建失败状态，按次保留输出，不清空历史日志。
build_log="$robot_root/logs/build/$(date +%Y%m%d-%H%M%S)-$$-$1.log"
echo "构建日志: $build_log"
build_selected "$1" 2>&1 | tee "$build_log"
