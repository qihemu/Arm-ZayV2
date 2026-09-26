# robot_wheel_control

> 2026-09-26统一配置更新：当前入口、参数和命令以[统一配置与落地调试](docs/unified_configuration.md)为准；下文旧bench数值保留作历史说明。

新增定距、定角与悬空轮角闭环，使用方法见[相对运动控制](docs/relative_motion.md)。CLI入口为`move`、`turn`、`wheel-angle`；服务为`/base/move_relative`，任务状态与误差通过`/base/state`发布。算法位于`relative_motion.hpp`与`wheel_relative_motion.cpp`，独占TX轮速来源。

日常操作请优先查阅[H55真机常用控制指令](../../../docs/H55真机常用控制指令.md)，包含完整启动、四方向、停车、状态、参数及故障恢复说明。

H55双轮差速底盘独立包，2026-09-25已实现配置解析、C++传输/运行层、ros2_control插件、ROS接口、台架节点与CLI。软件及模拟链路已通过测试；真实双轮新地址已核验，修复SDK发送队列竞争后，**真实ROS前后左右0.1rad/s短测通过；连续多圈及带载停车仍待验收**。

## 模块与运行边界

- 共享`damiao_core`，在`include/damiao_core/h55/`和`src/h55/`新增H55协议、寄存器与双轮事务；在`transports/`新增动态加载原厂SDK的传输适配。机械臂原protocol/registers/bus源码、机械限位和模式语义保持。
- 本包`WheelRuntime`负责RX、TX、管理三个线程，ROS API使用三线程执行器和分离回调组。服务仅入队，结果通过事件或查询服务获得。
- `H55BaseHardware`将轮关节position/velocity接到标准`diff_drive_controller`。底盘使用独立`/base/controller_manager`进程；机械臂使用另一个进程和独立CAN设备。共享库不意味着共享总线对象。
- `IWheelBackend`界定后端边界，当前DirectCanWheelBackend支持原厂USB SDK和SocketCAN；没有实现MCU固件或MCU通信协议。
- 控制器激活不使能电机。显式使能后仍须收到新指令，停机撤销运动许可；故障不会自动恢复运动。

完整core修改边界见`../damiao_core/docs/h55-module-boundaries.md`。新增H55文件按模块维护，不为轮子全局放宽机械臂限制。

## 配置与现有硬件

唯一源码配置位于仓库根`config/robot_wheel_control.yaml`，是应用配置，不能直接传给ROS `--params-file`。启动参数`config_file`可指定绝对路径，所有配置均在停机后重建会话生效。修改YAML不会改写电机寄存器。

2026-09-25 23:09/23:10逐台读回：左ESC/MST **13/45（0x0D/0x2D）**，右**14/46（0x0E/0x2E）**；两台模式3、CAN1M、固件ASCII 6920、子版本原始值56/55。商家工具CAN ID按十六进制显示，应输入0D/0E，而非13/14。未独立核验断电保持。

23:35已临时设置并读回实机TIMEOUT=4000、MAX_SPD=1；YAML要求TIMEOUT=4000（50us单位，200ms）与MAX_SPD=1rad/s。这些是低速台架保护候选值，启动只读核验，配置不一致会拒绝使能，不会自动写参数或保存Flash。SDK路径保持空，因为现有原厂SDK的控制时效尚未验收；序列号已填写实测适配器。

当前左右半径以图纸名义0.0705m初始化，尚未做带载滚动标定；轮距0.49m来自用户确认的胎面中线距离。null停稳阈值、回绕周期表示未标定，不是零。真实base/relative落地模式要求几何、多圈位置、带载停车验收；bench模式只开放原有轮角行程内的悬空等效目标，不能把结果当实车位移。运行程序没有模拟后端。

## 真机悬空测试操作

launch默认direct_usb_sdk，仅允许direct_usb_sdk/socketcan，不会静默回退到模拟。2026-09-26按用户要求，验证后已移除本包测试源码、假传输及CMake测试入口；本地构建脚本默认关闭BUILD_TESTING，上游原有测试保持原样。验证日志和退役测试源码保存在外层资料工作区，不进入Arm-ZayV2的Git变更。

保持两轮悬空固定，并关闭之前的轮控节点和商家调试工具。当前步骤使用哈希校验过的低延迟诊断SDK，适用于本次有人看护的短测，尚非正式部署SDK。

代码已构建；以后更改源码再运行`bash build_robot.sh wheel`。本机通过~/.cache/h55-wheel-control构建，以避开Humble rosidl中文路径问题；控制依赖位于外层.local/wheel-ros-deps。

终端1，准备保护参数并启动真实设备：

```bash
cd /home/G001/WorkSpace/轮式底盘/Arm-ZayV2
source scripts/wheel_real_bench_env.bash
python3 ../scripts/h55_dual_speed_test.py --stage link --address-profile migrated --run &&
ros2 launch robot_wheel_control wheel_bench.launch.py backend:=direct_usb_sdk config_file:="$H55_BENCH_CONFIG"
```

环境脚本只配置SDK依赖搜索路径，并根据根YAML生成~/.cache/h55-wheel-control/bench-diagnostic.yaml，不打开设备。诊断使用独立ROS_DOMAIN_ID=55，所有终端都须source同一脚本。

`link`是真机准备：核对13/45、14/46、模式/量化/供电，先确认失能，临时写入TIMEOUT=4000与MAX_SPD=1并读回，随后在失能下检查100Hz链路。它不发送使能或轮速，不改ID，不保存Flash。断电后需要重做；失败返回非零，不会执行后面的launch。共享设备锁会阻止与本包另一个SDK进程同时打开适配器。

终端2，查看状态，然后**逐条执行**：

```bash
cd /home/G001/WorkSpace/轮式底盘/Arm-ZayV2
source scripts/wheel_real_bench_env.bash
ros2 run robot_wheel_control wheel_cli status
ros2 run robot_wheel_control wheel_cli forward --enable --speed 0.1 --duration 1
ros2 run robot_wheel_control wheel_cli backward --enable --speed 0.1 --duration 1
ros2 run robot_wheel_control wheel_cli left --enable --speed 0.1 --duration 1
ros2 run robot_wheel_control wheel_cli right --enable --speed 0.1 --duration 1
ros2 run robot_wheel_control wheel_cli stop
```

status应显示backend=direct_usb_sdk、operation_mode=bench、lifecycle_state=1，且两轮feedback_valid=true。FAULT/保护不符/超时均应先排查，不能去掉保护条件继续。每条命令结束后自动停车并失能，下条命令的--enable是新的显式许可。

0.1rad/s很慢，1秒约转几度，且包含加速过程；悬空时只能观察轮子转向，车体不会实际平移。逻辑前进为两轮正转，电机原始方向映射左负右正。连续角和距离尚未验收，因此状态中的连续轮角/里程可以为NaN；目前优先看raw_position_rad与raw_velocity_rad_s。

终端3可在发送运动命令之前观察实时反馈：

```bash
cd /home/G001/WorkSpace/轮式底盘/Arm-ZayV2
source scripts/wheel_real_bench_env.bash
ros2 topic echo /base/state
```

当前停稳阈值未标定，结束时可能报告“Drivers disabled; physical standstill is not confirmed”，意思是已收到失能反馈，但软件还不能给出物理停稳认证，不应误报为已完成带载停车验收。

USB重新插拔后临时ACL可能丢失。用`lsusb -d 34b7:6877`查看当前Bus/Device，然后对对应`/dev/bus/usb/BBB/DDD`执行`sudo setfacl -m u:G001:rw ...`。当前已有权限时无需再次设置。

真实base模式仍须完成有效轮径/轮距、多圈位置与带载停车验收；此阶段使用wheel_bench，不启用base模式的里程计。

## 只读验证

停止轮控进程后可运行`ros2 run robot_wheel_control wheel_readback "$H55_BENCH_CONFIG"`，它使用同一C++传输和H55解码读取28项参数，不调用使能、FD、速度或写参。诊断读参期限1秒，不放宽运动线程的期限。

## 对外接口与黑板

| ROS名称 | 接口与用途 |
| --- | --- |
| `/base/wheel_velocity` | WheelVelocityCommand，bench专用；带会话和新鲜时间戳 |
| `/base/cmd_vel` | TwistStamped，base专用，经许可检查后转给差速控制器 |
| `/base/state` | WheelBaseState，10Hz状态黑板：左右反馈、原始/连续轮角、速度、温度、扭矩、样本年龄、故障、里程有效性 |
| `/base/control_events` | WheelControlEvent，异步管理结果 |
| `/base/joint_states` | 标准轮关节状态 |
| `/base/wheel_odom` | base模式标准Odometry，轮编码器推算，不是绝对定位 |
| `/base/set_enabled` | SetWheelBaseEnabled，显式使能/失能 |
| `/base/stop` | StopWheelBase，专用优先停止槽；默认CLI要求失能 |
| `/base/clear_fault` | ClearWheelBaseFault，匹配会话/故障序号；不会自动使能 |
| `/base/get_state` | GetWheelBaseState，只读快照 |
| `/base/get_control_result` | GetWheelControlResult，以session/request ID查结果 |
| `/base/move_relative` | MoveWheelBaseRelative，提交相对距离/转角任务，显式使能 |
| `/base/relative_keepalive` | WheelMotionHeartbeat，任务心跳；不启动或重放任务 |

轮控消息与服务定义在`robot_interfaces`（5msg/6srv）。accepted只代表入队；同request_id同请求体在保留窗口内幂等。Stop(false)在确认停稳后可保持零速使能，但撤销运动许可；故障或无法确认停稳时仍失能。

状态消息的发布时间与传感器采集时间分开；反馈失联时样本年龄继续增长，不能用刚发布的旧数值认定新鲜。未知连续位置/距离为NaN。机械驻车、硬件急停与整车任务互锁尚未接入，因此parking_confirmed及arm_operation_permitted始终false；软件Stop不能替代硬件急停。

目前不发布odom TF，整车TF/IMU/激光定位融合另行接入。故障后硬件插件保留ROS查询能力并停止电机；标准wheel_odom可能保留最后轮状态，消费者必须同时检查`/base/state.odometry_valid`，不能把它视为仍有效定位。真实base故障恢复要求重启新会话，避免悄悄重置里程。

## 验证范围

- 新轮控制软件测试：正负帧、配置拒绝、位置回绕/重复/断档、身份/保护不符拒绝使能、双轮部分发送失败停车、四方向、指令过期、停机保持/失能与幂等、反馈丢失故障。
- 已完成离线回归及ROS/CLI端到端验证，涵盖相对运动、心跳、取消、无位移进展及目标拒绝；验证后测试源码与构建入口已移除。
- 原机械臂core/tools/hardware软件回归；vcan测试在当前环境跳过，不能计为已执行。
- 实机已完成参数核验、30秒失能连续反馈观察、ROS服务四方向低速短测及停机后观察。USB长时间负载、当前程序失联保护、多圈位置及落地效果待验收。
- 相对运动已通过真机悬空5°轮角、±1cm等效距离、±3°等效车体转角及最终失能确认；编码器等效结果不等于落地精度验收。

统一编译、启动和按包查看日志，见[工作空间脚本说明](../../scripts/README.md)；三个入口位于工作空间根目录。
