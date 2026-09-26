#!/usr/bin/env bash
# 仅查看本工作空间按功能包保存的日志，不启停进程、不清理日志、不调用sudo。
set -eo pipefail
robot_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
log_root="${ARM_ZAY_LOG_DIR:-$robot_root/logs/runtime}"
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    echo "用法: $0 [lidar_slam|robot_wheel_control] [--list|--follow] [--session 名称|--mode 模式] [--lines 行数]"
    echo '默认显示该包最新会话末尾100行；--follow持续跟随该次会话，Ctrl+C仅退出查看。'
    exit 0
fi
if [[ $# == 0 ]]; then
    echo '日志: 1) lidar_slam  2) robot_wheel_control  q)退出'
    read -r -p '选择: ' selection || exit 1
    case "$selection" in
        1) set -- lidar_slam --follow ;; 2) set -- robot_wheel_control --follow ;;
        q) exit 0 ;; *) echo '无效选择' >&2; exit 2 ;;
    esac
fi
package=$1
shift
case "$package" in lidar_slam|robot_wheel_control) ;; *) echo '不支持的功能包' >&2; exit 2 ;; esac
session=latest
lines=100
follow=false
list=false
# 严格验证会话名，防止路径跳出当前功能包目录。
while [[ $# -gt 0 ]]; do
    case "$1" in
        --follow) follow=true; shift ;;
        --list) list=true; shift ;;
        --session|--mode|--lines)
            [[ $# -ge 2 ]] || { echo '缺少选项值' >&2; exit 2; }
            case "$1" in
                --session) session=$2 ;;
                --mode) session="latest-$2" ;;
                --lines) lines=$2 ;;
            esac
            shift 2 ;;
        *) echo "未知选项: $1" >&2; exit 2 ;;
    esac
done
[[ "$session" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ && "$lines" =~ ^[1-9][0-9]*$ ]] || { echo '会话名或行数无效' >&2; exit 2; }
directory="$log_root/$package"
[[ -d "$directory" ]] || { echo "该包尚无日志: $directory" >&2; exit 1; }
if "$list"; then
    find "$directory" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort -r
    exit 0
fi
file="$directory/$session/console.log"
[[ -f "$file" ]] || { echo "找不到会话日志: $file" >&2; exit 1; }
# 解析latest一次：跟随固定会话，避免新启动后悄然混入不同会话。
file=$(readlink -f "$file")
echo "查看: $file"
if "$follow"; then exec tail -n "$lines" -F "$file"; else exec tail -n "$lines" "$file"; fi
