# source本文件进入本机轮控构建环境，不启动节点、不访问硬件。
wheel_env_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
wheel_env_deps="$(dirname "$wheel_env_root")/.local/wheel-ros-deps/root"
wheel_env_build="${XDG_CACHE_HOME:-$HOME/.cache}/h55-wheel-control"
source /opt/ros/humble/setup.bash
if [[ -d "$wheel_env_deps/opt/ros/humble" ]]; then
    export AMENT_PREFIX_PATH="$wheel_env_deps/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
    export CMAKE_PREFIX_PATH="$wheel_env_deps/opt/ros/humble:$wheel_env_deps/usr:${CMAKE_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="$wheel_env_deps/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
    export PYTHONPATH="$wheel_env_deps/opt/ros/humble/local/lib/python3.10/dist-packages:$wheel_env_deps/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
fi
if [[ -f "$wheel_env_build/install/local_setup.bash" ]]; then
    source "$wheel_env_build/install/local_setup.bash"
fi
unset wheel_env_root wheel_env_deps wheel_env_build
