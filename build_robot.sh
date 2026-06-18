#!/bin/bash
# 选择性编译脚本 - 允许用户选择需要编译的包

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="${SCRIPT_DIR}"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印标题
echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  按摩机器人选择性编译脚本${NC}"
echo -e "${BLUE}================================${NC}"
echo ""



# Source ROS2 setup
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${GREEN}✓ Sourcing ROS2 Humble setup...${NC}"
    source /opt/ros/humble/setup.bash
else
    echo -e "${RED}错误: ROS2 Humble setup.bash 未找到${NC}"
    exit 1
fi

cd "${WORKSPACE_DIR}"

# 获取所有包
echo -e "${YELLOW}正在扫描工作空间中的包...${NC}"
echo ""

# 获取所有包列表
PACKAGES=($(colcon list -n 2>/dev/null))

if [ ${#PACKAGES[@]} -eq 0 ]; then
    echo -e "${RED}错误: 未找到任何包${NC}"
    exit 1
fi

# 显示包列表
echo -e "${GREEN}找到以下包:${NC}"
echo ""

# 创建包数组用于选择
declare -a SELECTED_PACKAGES=()
declare -a PACKAGE_STATUS=()

# 初始化所有包状态为未选择
for i in "${!PACKAGES[@]}"; do
    PACKAGE_STATUS[$i]=0
done

# 显示菜单函数
show_menu() {
    # 保留历史输出日志，避免每次刷新菜单时清屏。
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}  选择需要编译的包${NC}"
    echo -e "${BLUE}================================${NC}"
    echo ""
    
    for i in "${!PACKAGES[@]}"; do
        local num=$((i + 1))
        if [ ${PACKAGE_STATUS[$i]} -eq 1 ]; then
            echo -e "${GREEN}[$num] [✓] ${PACKAGES[$i]}${NC}"
        else
            echo -e " $num  [ ] ${PACKAGES[$i]}"
        fi
    done
    
    echo ""
    echo -e "${YELLOW}操作说明:${NC}"
    echo "  - 输入数字选择/取消选择包"
    echo "  - 输入 'a' 或 'all' 选择所有包"
    echo "  - 输入 'n' 或 'none' 取消所有选择"
    echo "  - 输入 'b' 或 'build' 开始编译"
    echo "  - 输入 'q' 或 'quit' 退出"
    echo ""
    echo -n "请选择: "
}

# 主循环
while true; do
    show_menu
    read -r choice
    
    case "$choice" in
        a|all)
            # 选择所有包
            for i in "${!PACKAGES[@]}"; do
                PACKAGE_STATUS[$i]=1
            done
            ;;
        n|none)
            # 取消所有选择
            for i in "${!PACKAGES[@]}"; do
                PACKAGE_STATUS[$i]=0
            done
            ;;
        b|build)
            # 开始编译
            SELECTED_PACKAGES=()
            for i in "${!PACKAGES[@]}"; do
                if [ ${PACKAGE_STATUS[$i]} -eq 1 ]; then
                    SELECTED_PACKAGES+=("${PACKAGES[$i]}")
                fi
            done
            
            if [ ${#SELECTED_PACKAGES[@]} -eq 0 ]; then
                echo ""
                echo -e "${RED}错误: 没有选择任何包!${NC}"
                echo ""
                read -p "按回车继续..."
                echo ""
                continue
            fi
            
            # 检查是否选择了所有包
            IS_ALL_PACKAGES=true
            if [ ${#SELECTED_PACKAGES[@]} -ne ${#PACKAGES[@]} ]; then
                IS_ALL_PACKAGES=false
            fi
            
            # 对选中的包进行排序，确保基础包优先编译
            # 优先级: robot_interfaces > robot_ros_description > 其他包
            PRIORITY_PACKAGES=()
            OTHER_PACKAGES=()
            
            for pkg in "${SELECTED_PACKAGES[@]}"; do
                if [ "$pkg" = "robot_interfaces" ]; then
                    PRIORITY_PACKAGES=("robot_interfaces" "${PRIORITY_PACKAGES[@]}")
                elif [ "$pkg" = "robot_ros_description" ]; then
                    # robot_ros_description 排在 robot_interfaces 之后
                    if [[ " ${PRIORITY_PACKAGES[@]} " =~ " robot_interfaces " ]]; then
                        # 如果已经有 robot_interfaces，插入到它后面
                        PRIORITY_PACKAGES=("${PRIORITY_PACKAGES[@]}" "robot_ros_description")
                    else
                        # 否则放在最前面
                        PRIORITY_PACKAGES=("robot_ros_description" "${PRIORITY_PACKAGES[@]}")
                    fi
                else
                    OTHER_PACKAGES+=("$pkg")
                fi
            done
            
            # 合并排序后的包列表
            SELECTED_PACKAGES=("${PRIORITY_PACKAGES[@]}" "${OTHER_PACKAGES[@]}")
            
            clear
            echo -e "${BLUE}================================${NC}"
            echo -e "${BLUE}  开始编译选定的包${NC}"
            echo -e "${BLUE}================================${NC}"
            echo ""
            echo -e "${GREEN}将要编译以下包:${NC}"
            for pkg in "${SELECTED_PACKAGES[@]}"; do
                echo "  - $pkg"
            done
            echo ""
            
            # 如果是全部编译，提示将先清理
            if [ "$IS_ALL_PACKAGES" = true ]; then
                echo -e "${YELLOW}注意: 全部编译模式将先清理 build、install 和 log 目录${NC}"
                echo ""
            fi

            # 询问编译模式
            echo ""
            echo -n "请选择编译模式 (1=Release, 2=Debug) [默认: 1]: "
            read -r build_mode_choice
            
            BUILD_TYPE="Release"
            if [[ "$build_mode_choice" == "2" ]]; then
                BUILD_TYPE="Debug"
                echo -e "${YELLOW}✓ 使用 Debug 模式编译 (包含调试信息)${NC}"
            else
                BUILD_TYPE="Release"
                echo -e "${GREEN}✓ 使用 Release 模式编译 (优化性能)${NC}"
            fi
            
            # 询问是否使用 --merge-install 参数
            echo ""
            echo -n "是否使用 --merge-install 参数，默认不使用? (y/N): "
            read -r merge_install_choice
            
            MERGE_INSTALL_FLAG=""
            if [[ "$merge_install_choice" =~ ^[yY]$ ]]; then
                MERGE_INSTALL_FLAG="--merge-install"
                echo -e "${GREEN}✓ 将使用 --merge-install 参数${NC}"
            else
                echo -e "${YELLOW}✓ 不使用 --merge-install 参数${NC}"
            fi

            # 询问 CPU 并行度
            TOTAL_CPUS=$(nproc)
            DEFAULT_WORKERS=$((TOTAL_CPUS > 1 ? TOTAL_CPUS - 1 : 1))
            echo ""
            echo -n "请输入并行编译包数量 (CPU核心数: ${TOTAL_CPUS}, 默认: ${DEFAULT_WORKERS}，直接回车使用默认值): "
            read -r parallel_choice

            PARALLEL_WORKERS_FLAG=""
            CMAKE_PARALLEL_LEVEL=""
            if [[ "$parallel_choice" =~ ^[1-9][0-9]*$ ]]; then
                PARALLEL_WORKERS_FLAG="--parallel-workers $parallel_choice"
                CMAKE_PARALLEL_LEVEL="-DCMAKE_BUILD_PARALLEL_LEVEL=$parallel_choice"
                echo -e "${GREEN}✓ 使用 $parallel_choice 个并行工作进程${NC}"
            else
                PARALLEL_WORKERS_FLAG="--parallel-workers $DEFAULT_WORKERS"
                CMAKE_PARALLEL_LEVEL="-DCMAKE_BUILD_PARALLEL_LEVEL=$DEFAULT_WORKERS"
                echo -e "${GREEN}✓ 使用默认 $DEFAULT_WORKERS 个并行工作进程${NC}"
            fi
            echo ""
            
            # 如果是全部编译，先清理
            if [ "$IS_ALL_PACKAGES" = true ]; then
                echo -e "${YELLOW}正在清理编译产物...${NC}"
                echo ""
                
                if [ -d "${WORKSPACE_DIR}/build" ]; then
                    echo "  - 删除 build 目录"
                    rm -rf "${WORKSPACE_DIR}/build"
                fi
                
                if [ -d "${WORKSPACE_DIR}/install" ]; then
                    echo "  - 删除 install 目录"
                    rm -rf "${WORKSPACE_DIR}/install"
                fi
                
                if [ -d "${WORKSPACE_DIR}/log" ]; then
                    echo "  - 删除 log 目录"
                    rm -rf "${WORKSPACE_DIR}/log"
                fi
                
                echo ""
                echo -e "${GREEN}✓ 清理完成${NC}"
                echo ""
            fi
            
            BUILD_STATUS=0
            
            # 检查是否需要优先编译基础包
            if [[ " ${PRIORITY_PACKAGES[@]} " =~ " robot_interfaces " ]] || [[ " ${PRIORITY_PACKAGES[@]} " =~ " robot_ros_description " ]]; then
                echo -e "${YELLOW}步骤 1: 优先编译基础包...${NC}"
                echo ""
                
                # 构建命令参数（--packages-select 后跟所有包名）
                echo -e "${YELLOW}执行命令: colcon build --symlink-install ${MERGE_INSTALL_FLAG} ${PARALLEL_WORKERS_FLAG} --packages-select ${PRIORITY_PACKAGES[*]} --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${CMAKE_PARALLEL_LEVEL}${NC}"
                echo ""
                
                colcon build --symlink-install ${MERGE_INSTALL_FLAG} ${PARALLEL_WORKERS_FLAG} --packages-select "${PRIORITY_PACKAGES[@]}" --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${CMAKE_PARALLEL_LEVEL}
                BUILD_STATUS=$?
                
                if [ $BUILD_STATUS -ne 0 ]; then
                    echo ""
                    echo -e "${RED}基础包编译失败! (错误码: $BUILD_STATUS)${NC}"
                else
                    echo ""
                    echo -e "${GREEN}✓ 基础包编译成功${NC}"
                    
                    # 如果还有其他包需要编译
                    if [ ${#OTHER_PACKAGES[@]} -gt 0 ]; then
                        echo ""
                        echo -e "${YELLOW}步骤 2: 编译其他包...${NC}"
                        echo ""
                        
                        echo -e "${YELLOW}执行命令: colcon build --symlink-install ${MERGE_INSTALL_FLAG} ${PARALLEL_WORKERS_FLAG} --packages-select ${OTHER_PACKAGES[*]} --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${CMAKE_PARALLEL_LEVEL}${NC}"
                        echo ""
                        
                        colcon build --symlink-install ${MERGE_INSTALL_FLAG} ${PARALLEL_WORKERS_FLAG} --packages-select "${OTHER_PACKAGES[@]}" --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${CMAKE_PARALLEL_LEVEL}
                        BUILD_STATUS=$?
                    fi
                fi
            else
                # 没有优先包，直接编译所有选中的包
                echo -e "${YELLOW}执行命令: colcon build --symlink-install ${MERGE_INSTALL_FLAG} ${PARALLEL_WORKERS_FLAG} --packages-select ${SELECTED_PACKAGES[*]} --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${CMAKE_PARALLEL_LEVEL}${NC}"
                echo ""
                
                colcon build --symlink-install ${MERGE_INSTALL_FLAG} ${PARALLEL_WORKERS_FLAG} --packages-select "${SELECTED_PACKAGES[@]}" --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${CMAKE_PARALLEL_LEVEL}
                BUILD_STATUS=$?
            fi
            echo ""
            if [ $BUILD_STATUS -eq 0 ]; then
                echo -e "${GREEN}================================${NC}"
                echo -e "${GREEN}  编译成功!${NC}"
                echo -e "${GREEN}================================${NC}"
            else
                echo -e "${RED}================================${NC}"
                echo -e "${RED}  编译失败! (错误码: $BUILD_STATUS)${NC}"
                echo -e "${RED}================================${NC}"
            fi
            
            echo ""
            read -p "按回车继续..."
            echo ""
            ;;
        q|quit)
            echo ""
            echo -e "${YELLOW}退出编译脚本${NC}"
            exit 0
            ;;
        ''|*)
            # 数字选择
            if [[ "$choice" =~ ^[0-9]+$ ]]; then
                idx=$((choice - 1))
                if [ $idx -ge 0 ] && [ $idx -lt ${#PACKAGES[@]} ]; then
                    # 切换选择状态
                    if [ ${PACKAGE_STATUS[$idx]} -eq 1 ]; then
                        PACKAGE_STATUS[$idx]=0
                    else
                        PACKAGE_STATUS[$idx]=1
                    fi
                else
                    echo ""
                    echo -e "${RED}无效的选择!${NC}"
                    sleep 1
                fi
            elif [ -n "$choice" ]; then
                echo ""
                echo -e "${RED}无效的输入!${NC}"
                sleep 1
            fi
            ;;
    esac
done
