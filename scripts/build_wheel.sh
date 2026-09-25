#!/usr/bin/env bash
# 在ASCII缓存目录构建，避免本机Humble rosidl的中文路径问题；不操作硬件。
set -eo pipefail
wheel_source_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
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
