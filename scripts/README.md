# 工作空间编译、启动与日志入口

三个用户入口位于 **Arm-ZayV2根目录**，可从任意当前目录调用；不需要sudo，不保存密码，不自动安装依赖。无参数时显示交互菜单，`--help`显示命令用法。

## 编译

```bash
cd /home/G001/WorkSpace/轮式底盘/Arm-ZayV2
./build_robot.sh all
./build_robot.sh wheel
./build_robot.sh lidar
./build_robot.sh core
```

`all`表示当前H55轮控与C1感知模块及共享依赖，不代表原机械臂、MoveIt等整个仓库。轮控构建5个包，雷达构建2个包（robot_interfaces为共同依赖）。保留原有ASCII缓存，分别为`~/.cache/h55-wheel-control`、`~/.cache/arm-zay-lidar`；单独core使用`~/.cache/arm-zay-core`。遵循XDG_CACHE_HOME。保留增量构建，不清空全部产物。轮控关闭测试编译，雷达保留已有测试编译配置；编译本身不执行测试。

构建完整输出保存在`logs/build/时间-PID-目标.log`，失败退出码会返回终端。同一工作空间只允许一个新入口构建进程。源码根`config/robot_wheel_control.yaml`与`config/lidar_slam.yaml`仍是用户维护的配置源。

## 启动

```bash
./start_robot.sh lidar serial_port:=/dev/rplidar
# 已有其他驱动占用串口时：禁用新驱动，并按配置说明调整输入话题/坐标系。
./start_robot.sh lidar driver:=false
./start_robot.sh wheel
# 以下在另一个终端使用；需要雷达数据及有效的odom->base TF。
./start_robot.sh mapping
./start_robot.sh localization map_directory:=/绝对路径/地图版本 initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0
./start_robot.sh amcl map_directory:=/绝对路径/地图版本
```

每次只前台启动一个模块。Ctrl+C正常结束该次launch。wheel调用现有`wheel_real_bench_env.bash`，保持哈希校验诊断SDK、ROS_DOMAIN_ID=55及本机通信；不执行link写参，不自动使能或发送速度。断电后的保护参数准备仍按轮控README单独执行，不能把启动成功当作运动验收。雷达与SLAM继承调用终端的ROS域（未设置时为0）；需要与轮控通信时，须明确统一ROS域，不自动开启尚未验收的base模式。

脚本防止同入口重复启动同类模块，并让mapping/localization/amcl互斥；锁不负责检测外部旧驱动或其他工作空间进程。启动前仍需关闭占用串口的原驱动。保留ROS launch原生参数，所有`参数:=值`按原样传递。

## 单功能包日志

```bash
./system_control.sh lidar_slam
./system_control.sh robot_wheel_control --follow
./system_control.sh lidar_slam --list
./system_control.sh lidar_slam --mode mapping --lines 200
./system_control.sh lidar_slam --session 20260926-120000-1234-lidar --follow
```

默认显示该包最新会话最后100行；`--list`列出历史会话，`--follow`跟随所选会话。Ctrl+C只退出日志查看，不停止运行节点。雷达采集与建图均属于lidar_slam，可用`--mode lidar/mapping/localization/amcl`选择各模式最近一次日志。

目录为`logs/runtime/<功能包>/<时间-PID-模式>/`，`console.log`保存launch及其子节点终端输出，`ros/`保存ROS原生日志。这里按**所启动功能包的launch会话**归档，包含其依赖节点，不声称按节点所属软件包逐条分类。可用`ARM_ZAY_LOG_DIR`指定绝对日志根目录；启动与查看需设置同一个值。跟随时固定到当次会话，新启动后需重新打开查看。日志不自动删除。此前手工执行的ros2命令不会被追溯收集，原日志仍在原ROS日志目录。

## scripts目录保留与移除

保留三个source辅助脚本：`wheel_env.bash`（轮控环境）、`wheel_real_bench_env.bash`（当前悬空真机环境及派生配置）、`lidar_env.bash`（雷达环境及根配置路径）。它们供新入口和独立ROS CLI终端共用。

原`build_wheel.sh`、`build_lidar.sh`、`build_damiao_core.sh`已由根目录`build_robot.sh`对应目标替代。`setup_can.sh`已移除：当前H55使用USB SDK；未来若用SocketCAN，须按实际接口单独配置1Mbps。旧`ros2_control_diagnose_once.sh`在此次修改开始前已不存在，本次未恢复它。

原参考脚本的其他机器人路径、OTA/systemd管理、密码、数据库及虚拟环境安装逻辑未引入本项目。当前脚本不提供机械臂一键使能入口。

## 本次验证（2026-09-26）

新入口`build_robot.sh all`完成轮控5包及雷达2包构建；Bash语法及Git差异格式检查通过。独立ROS域169、`driver:=false rviz:=false`启动扫描处理与文件服务，SIGINT后两个节点正常退出，关闭日志完整保存，按包查看可读。离线替身检查了启动/构建失败退出码、日志隔离、非法会话拒绝、缺失日志和带空格路径。未启动轮控硬件、未访问雷达串口，不替代真机运动或建图验收。
