# H55 统一配置与落地调试

更新：2026-09-26。本版实现运行能力不等于所有速度和载荷已验收；现场结果见工作空间 `docs/H55统一配置与落地调试记录.md`。当前车架约4kg，最终整车预计30kg。

## 唯一配置源和启动

唯一人工编辑入口是工程根 `config/robot_wheel_control.yaml`。先停车并确认新鲜失能反馈，再退出节点、编辑、重启。没有热加载。启动不会自动使能。

```bash
cd /home/G001/WorkSpace/轮式底盘/Arm-ZayV2
./start_robot.sh wheel
```

环境脚本保留名字 `scripts/wheel_real_bench_env.bash` 以兼容已有终端。它只加载环境、核验诊断SDK并派生SDK路径；不覆盖源YAML的backend/mode。缓存 `~/.cache/h55-wheel-control/bench-diagnostic.yaml` 不是第二个配置入口。

启动先核验两轮身份/模式/量化范围，再确认失能和电压温度，按 `write_on_startup` 写入TIMEOUT/MAX_SPD，逐项读回并再次失能确认。任何失败都禁止使能。只在值不符时写入，失败不自动重试或回滚，不保存Flash。ID/模式/PMAX/VMAX/TMAX不会自动写入。其他YAML保护项是主机检查，不能当作已写入驱动。

`wheel_prepare "$H55_BENCH_CONFIG"` 和历史 `h55_dual_speed_test.py --stage link --address-profile migrated --run` 使用相同C++初始化逻辑。运行节点时不能另开prepare。历史固定速度运动阶段已停用，其文本保存在工作空间历史记录中。`wheel_readback` 保持只读。

## 参数位置与含义

以下均在 `robot_wheel_control:` 下；null仅在明确支持的字段中表示不设业务上限，不允许用0代替。

| 字段 | 单位/当前值 | 作用 |
|---|---|---|
| `operation_mode` | commissioning | 独立低速落地调试；bench悬空；relative正式定距/手动；base由差速控制器唯一写入 |
| `limits.max_wheel_speed_rad_s` | 6 rad/s | 全局命令上限；名义车速0.423m/s，不代表本轮已验证 |
| `commissioning.max_wheel_speed_rad_s` | 1 rad/s | 当前模式与全局取较小值；首测使用0.5 |
| `commissioning.max_action_distance_m` | 1 m | 左右各自累计绝对路程取较大者，含转向、回退及停车 |
| `commissioning.stop_margin_m` | 0.10 m | 除动态制动距离和通信反应距离之外的余量 |
| `commissioning.raw_position_margin_rad` | 0.20 rad | 未验收回绕时避免接近原始反馈边界；不能当回绕周期 |
| `bench.max_wheel_speed_rad_s` / `limits.bench_max_travel_rad` | 0.2 rad/s / 1 rad | 悬空模式单独限制 |
| `limits.max_wheel_acceleration_rad_s2` | 1 rad/s² | 正常轮加速度 |
| `limits.max_wheel_deceleration_rad_s2` | 2 rad/s² | 正常轮减速度，反向先过零；非失联物理制动保证 |
| `limits.max_linear_speed_m_s` | 0.42 m/s | 车体线速度上限，仍受轮速限制 |
| `limits.max_angular_speed_rad_s` | 0.5 rad/s | 车体偏航速度，不是轮速 |
| `limits.max_linear_acceleration_m_s2` / `max_linear_deceleration_m_s2` | 0.07 / 0.14 m/s² | 车体线加/减速度 |
| `limits.max_angular_acceleration_rad_s2` / `max_angular_deceleration_rad_s2` | 0.2 / 0.4 rad/s² | 车体转向加/减速度 |
| `driver_protection.max_speed_rad_s` | 6 rad/s | 启动核验/写入驱动MAX_SPD |
| `driver_protection.timeout_register_raw` | 4000×50μs=200ms | 启动核验/写入驱动TIMEOUT |
| `driver_protection.write_on_startup` / `save_to_flash_on_startup` | true / false | 仅允许易失保护初始化，不允许自动存Flash |
| `driver_protection.min_bus_voltage_v` / `max_bus_voltage_v` | 22 / 26 V | 初始化电压检查，当前24V供电 |
| `limits.max_reported_torque_nm` | 0.8 N·m | 主机反馈扭矩监视，非扭矩控制命令 |
| `limits.max_motor_temperature_c` / `max_driver_temperature_c` | 45 / 45℃ | 主机温度保护 |
| `timing.command_timeout_ms` / `feedback_timeout_ms` | 150 / 50ms | 上游指令/运动反馈失联期限，长程不放宽 |
| `timing.stop_confirmation_timeout_ms` | 1000ms | 速度降至零所需时间之外的停车确认余量 |
| `relative_motion.max_distance_m` | null | 无业务距离上限；调试预算、原始位置边界仍优先 |
| `relative_motion.max_timeout_s` | null | 无固定业务时间上限；每个任务仍有有限期限 |
| `relative_motion.timeout_factor` / `timeout_margin_s` | 2 / 5s | 自动期限=含加减速/到位保持的估计时间×2+5s |
| `relative_motion.heartbeat_timeout_ms` | 500ms | 定距控制端失联后停车 |
| `operator.*` | 默认轮速0.5、线速0.035、转向0.1、时长1s、发送20Hz | CLI从节点服务获取默认值；`max_duration_s: null`不设定时业务上限 |

速度、加减速在轮级和车体级同时约束；差速双轮共同缩放保持比例。方向系数与轮径仍在 `wheels.left/right`，轮距在 `geometry.wheel_separation_m`。当前方向按源YAML保留，不用历史文档覆盖用户修改。

`commissioning`并不把 `geometry_calibrated/loaded_stop_verified/continuous_position_verified` 改为true。没有回绕验收时仅允许不跨原始反馈范围的短动作；边界附近可能在1米之前就拒绝/停车。连续多圈和地面标定完成后才使用正式模式；不能用PMAX直接填wrap_period_rad。

## 控制命令

第二终端先加载环境；以下是有人看护的调试命令，不是自动执行清单。

```bash
cd /home/G001/WorkSpace/轮式底盘/Arm-ZayV2
source scripts/wheel_real_bench_env.bash
ros2 run robot_wheel_control wheel_cli configuration
ros2 run robot_wheel_control wheel_cli status

# 编码器前进/后退10cm；速度m/s；未指定timeout由服务器计算。
ros2 run robot_wheel_control wheel_cli move --distance 0.10 --linear-speed 0.035 --enable
ros2 run robot_wheel_control wheel_cli move --distance -0.10 --linear-speed 0.035 --enable
# 车体转角度；转向速度rad/s。
ros2 run robot_wheel_control wheel_cli turn --angle 10 --angular-speed 0.10 --enable
ros2 run robot_wheel_control wheel_cli turn --angle -10 --angular-speed 0.10 --enable

# 方向命令的speed是轮rad/s；commissioning仍按单动作预算自动停车。
ros2 run robot_wheel_control wheel_cli forward --speed 0.5 --continuous --enable
# 或定时运行；duration和continuous互斥。
ros2 run robot_wheel_control wheel_cli backward --speed 0.5 --duration 5 --enable

ros2 run robot_wheel_control wheel_cli stop
ros2 run robot_wheel_control wheel_cli status
```

定距任务结束后停车失能，下一次仍需 `--enable`。单独enable后若没有新指令，150ms指令期限同样触发停车；推荐在运动命令上使用--enable，不提前单独使能等待。持续运动遇Ctrl+C、显式stop、控制端失联、行程保护或故障结束；没有“无限距离绕过调试限制”。不要反复enable重置活动预算：运行层拒绝重复使能。失能不等于抱闸驻车，不授予双臂作业许可。

## 接口与兼容

- 新增 `/base/get_configuration` (`GetWheelConfiguration`)：返回session、配置摘要、读取路径、实际生效YAML（其中轮速为当前模式有效值），不产生CAN事务。
- `WheelVelocityCommand.source_id`：每控制客户端唯一ID，同一次使能首个来源持有控制权直到停止；session和新鲜时间戳仍必需。旧发布器需要补充此字段。
- `MoveWheelBaseRelative.timeout_s=0`：自动计算；正数是显式期限，过短/超配置/不可表示值拒绝。
- `WheelBaseState.action_wheel_travel_m`：两轮单动作绝对路程，包含停车；`action_distance_limit_m`：当前调试预算；`relative_timeout_s`：实际任务期限。
- 接口变更后需重建共享 `robot_interfaces` 的相关工作空间，退出旧节点再启动；不能把旧消息布局与新节点混用。
- `wheel_bench.launch.py`、`wheel_relative.launch.py` 都尊重源YAML模式；`wheel_base.launch.py`要求YAML明确为base，不自动覆盖。原名字仅作兼容入口。

## 当前验证范围

独立减速度1/2rad/s²是本轮初值；30kg、6rad/s、带载急停距离、诊断SDK长期运行仍需单独验收。按名义轮径，1rad/s²加速至6rad/s再以2rad/s²停车约需1.90m，故本次1m动作预算不验证满速。编码器距离不能消除打滑或证明物理活动边界，现场必须对照地面参照观察。

本轮4kg车架实测：前后±0.10m/±0.30m、左右±10°/±15°、主动stop和停止刷新指令均完成，最后新鲜双轮失能、故障0；单动作最大累计轮路程约0.30086m。采样轮速存在超调（最高1.14774rad/s），不能把0.993rad/s目标当精确反馈上限。未验证6rad/s、30kg、多圈或物理制动精度，正式验收标志保持false。
