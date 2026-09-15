#!/usr/bin/env bash
set -euo pipefail

# 根据脚本位置定位工作区，允许从任意目录调用。
damiao_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
damiao_workspace_dir="$(cd -- "${damiao_script_dir}/.." && pwd)"
damiao_build_dir="${damiao_workspace_dir}/build/damiao_core"

if ! command -v cmake >/dev/null 2>&1; then
    echo "错误：未找到 cmake，请先安装 CMake。" >&2
    exit 1
fi

# 独立配置并编译核心库；额外参数传递给 CMake，例如 -DCMAKE_BUILD_TYPE=Debug。
cmake -S "${damiao_workspace_dir}/src/damiao_core" \
    -B "${damiao_build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    "$@"
cmake --build "${damiao_build_dir}" --parallel

echo "damiao_core 编译完成：${damiao_build_dir}"
