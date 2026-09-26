# H55 相对距离与转角控制

> 2026-09-26统一配置更新：当前入口、参数和命令以[统一配置与落地调试](unified_configuration.md)为准；下文旧bench数值保留作历史说明。

本功能使用编码器位置反馈闭环控制轮速，接近目标减速，到位后停车、失能并观察最终位置。电机仍使用速度模式3，不切换驱动固件模式，不以“速度×时间”认定到位。

## 当前参数与边界

- 左右胎面中线间距：`0.49 m`，2026-09-26用户确认。固定螺纹孔安装面之间的`0.40 m`不用于差速计算。
- 手册第9页轮径`φ141±2 mm`，初始左右半径使用`0.0705 m`。这是图纸名义值，尚非带载有效滚动半径。
- 当前`bench`为双轮悬空模式。`move/turn`在此模式只验证编码器等效距离/转角，不代表底盘真的走了对应距离。
- 当前每轮目标不超过台架行程1rad的80%，留出停止余量。因此本组几何下，悬空`move`最多约`0.0564 m`，`turn`最多约`13.19°`，`wheel-angle`最多约`45.84°`。不能用连续小指令代替未完成的落地验收。
- 距离与转角误差来自轮编码器。打滑、轮胎压缩和侧向偏移不能只靠编码器消除；本功能不是地图导航或全局坐标定位。

## 启动与当前可用命令

源码改变后退出旧节点，再构建。每个终端分别加载环境：

```bash
cd /home/G001/WorkSpace/轮式底盘/Arm-ZayV2
bash build_robot.sh wheel
source scripts/wheel_real_bench_env.bash
```

节点未运行、双轮悬空固定时，终端一执行保护准备和启动。此准备脚本临时设置TIMEOUT=4000、MAX_SPD=1，不保存Flash：

```bash
python3 ../scripts/h55_dual_speed_test.py --stage link --address-profile migrated --run &&
ros2 launch robot_wheel_control wheel_bench.launch.py backend:=direct_usb_sdk config_file:="$H55_BENCH_CONFIG"
```

终端二加载同一环境，逐条执行：

```bash
cd /home/G001/WorkSpace/轮式底盘/Arm-ZayV2
source scripts/wheel_real_bench_env.bash
ros2 run robot_wheel_control wheel_cli status

# 前进等效1厘米，后退等效1厘米；米和米/秒。
ros2 run robot_wheel_control wheel_cli move --distance 0.01 --linear-speed 0.005 --timeout 20 --enable
ros2 run robot_wheel_control wheel_cli move --distance -0.01 --linear-speed 0.005 --timeout 20 --enable

# 左转等效3度，右转等效3度；--angle为度，--angular-speed为车体rad/s。
ros2 run robot_wheel_control wheel_cli turn --angle 3 --angular-speed 0.03 --timeout 20 --enable
ros2 run robot_wheel_control wheel_cli turn --angle -3 --angular-speed 0.03 --timeout 20 --enable

# 不依赖轮径的双轮逻辑转角：两轮各向前转5度。
ros2 run robot_wheel_control wheel_cli wheel-angle --angle 5 --speed 0.1 --timeout 15 --enable

ros2 run robot_wheel_control wheel_cli stop
```

每个相对任务从失能状态开始，必须带`--enable`，不能先运行单独的`enable`再提交相对任务。CLI会持续发送任务心跳，显示当前测量和误差，成功后输出`Relative target reached; drivers disabled`。失败返回非零退出码，不应把`accepted`当作已到位。

`Ctrl+C`会先尝试停车，再退出。进程被强杀或通信中断时，服务器的任务心跳期限触发停车。`stop`取消活动任务；它不是实体急停，也不自动清除驱动故障。

## 速度和误差

| 参数 | 单位与作用 |
| --- | --- |
| `move --distance` | 有符号米；正前进，负后退 |
| `move --linear-speed` | 最大车体线速度m/s；当前CLI上限0.01 |
| `turn --angle` | 有符号角度（度）；正左转，负右转 |
| `turn --angular-speed` | 最大车体角速度rad/s；当前CLI上限0.05 |
| `wheel-angle --angle` | 双轮逻辑转角（度），不是车体转角 |
| `wheel-angle --speed` | 轮角速度rad/s；当前上限0.2 |
| `--timeout` | 运动期限，默认120s，并受YAML最大期限180s限制；停止与最终确认另有有界期限 |

提高对应的速度参数可加快运动，例如`move --linear-speed 0.01`或`turn --angular-speed 0.05`。实际轮速始终受`max_wheel_speed_rad_s`限制，按比例缩放两轮速度，不因单轮截断改变运动比例。`--duration`仅属于旧的定时轮速命令，不能用它控制相对任务距离。

YAML `relative_motion`提供比例增益、同步增益、轮角误差、稳定速度、稳定时间、心跳期限、无进展期限、最大距离/转角和任务期限。当前候选最终轮角误差为`0.01 rad`，失能前先收敛到一半误差窗口，速度绝对值不高于`0.08 rad/s`，连续稳定`500 ms`；驱动失能后在有界期限内再次确认连续500ms满足最终误差和速度条件。按名义几何，最终轮角误差对应直线约0.705mm、车体转角约0.165°，这是编码器判据换算，**不是实车精度承诺**。

目标过小（每轮目标不大于两倍轮角误差）会在使能前拒绝，避免容差覆盖整个目标。无位移进展、超时、心跳过期、反馈失联、连续位置丢失或主动stop都使任务退出并尝试停车；通信错误仍按原驱动故障策略处理。

## 外部ROS接口

- `/base/move_relative`：`robot_interfaces/srv/MoveWheelBaseRelative`，异步提交并独占轮速源。kind=1距离m，2车体转角rad，3悬空双轮角度rad；服务角度一律用rad，与CLI的`--angle`度不同。
- `/base/relative_keepalive`：`robot_interfaces/msg/WheelMotionHeartbeat`，建议20Hz，携带新时间戳、session_id、request_id。过期、重放和旧会话心跳不续期。
- `/base/get_control_result`：用相同session_id/request_id查询。PENDING=1、SUCCEEDED=2、FAILED=3、CANCELLED=4。
- `/base/stop`：取消任务并停车。任务期间的手动轮速消息不会接管目标；另一个相对任务或使能请求会被拒绝。
- `/base/state`：新增`relative_motion_active`、`relative_request_id`、`relative_kind`、`relative_target`、`relative_measured`、`relative_error`、`relative_wheel_target_rad`、`relative_wheel_travel_rad`。结束后保留最后一次结果对应的测量值。

观测命令：

```bash
ros2 topic echo /base/state
ros2 interface show robot_interfaces/srv/MoveWheelBaseRelative
```

任务执行位于运行层唯一TX线程；服务回调只提交队列，RX和状态服务保持独立。新增算法在`relative_motion.hpp`及`wheel_relative_motion.cpp`中，未改变机械臂协议或硬件插件。

## 落地后的相对运动

完成以下实测后再启用`relative`模式，不能只把标志改成true：

1. 验证多圈位置的回绕/饱和规律和连续展开；填入实测`wrap_period_rad`并确认`continuous_position_verified`。
2. 带载测量有效滚动半径，校正左右轮及0.49m初始轮距模型，再确认`geometry_calibrated`。
3. 实测带载停车、停止位置窗口与速度阈值，确认`loaded_stop_verified`并填写`stopping`参数。
4. 填写车体速度/加速度限制；提高待机刷新率（例如100Hz），使采样间隔及调度裕量满足50ms连续位置约束。

退出bench节点，再用同一明确配置启动独立相对控制进程：

```bash
ros2 launch robot_wheel_control wheel_relative.launch.py backend:=direct_usb_sdk config_file:="$H55_BENCH_CONFIG"
```

此launch显式选择relative模式，不加载另一个diff_drive_controller写轮速。它与bench、base节点互斥运行。当前诊断SDK仍需部署时效验收。

上述验收完成后，以下才是**落地使用示例**，当前悬空模式会拒绝这些大目标：

```bash
ros2 run robot_wheel_control wheel_cli move --distance 0.5 --linear-speed 0.01 --timeout 90 --enable
ros2 run robot_wheel_control wheel_cli turn --angle 90 --angular-speed 0.05 --timeout 60 --enable
```

到位后失能不等于抱闸锁轮，也不授予双臂作业许可；正式驻车与外部定位仍需后续集成。
