# 环境入口不启动硬件，也不覆盖用户选定的ROS_DOMAIN_ID。
source /opt/ros/humble/setup.bash
# 默认读取源码根config，调整YAML无需重编译；显式环境覆盖继续保留。
lidar_env_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export ARM_ZAY_CONFIG_DIR="${ARM_ZAY_CONFIG_DIR:-$lidar_env_source/config}"
lidar_env_build="${XDG_CACHE_HOME:-$HOME/.cache}/arm-zay-lidar"
if [[ -f "$lidar_env_build/install/local_setup.bash" ]]; then
    source "$lidar_env_build/install/local_setup.bash"
fi
unset lidar_env_build lidar_env_source
