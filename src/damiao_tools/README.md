# damiao_tools

基于 `damiao_core` 的单 CAN 总线多电机交互式工具。启动后自动扫描配置接口上的电机，
注册全部已发现轴（最多 6 台，不限控制模式），并提供数字菜单进行状态查询、维护与
位置速度驱动。

## 构建

```bash
colcon build --packages-up-to damiao_tools
source install/setup.bash
```

工具依赖系统 `yaml-cpp`。示例配置中的运动限制是占位符，必须按台架实际情况填写：

```bash
damiao_motor_tool --file src/damiao_tools/config/motor.example.yaml
```

## 配置

```yaml
can_interface: can0
min_output_position_rad: -12.5
max_output_position_rad: 12.5
max_output_speed_rad_s: 3
scan_esc_min: 1
scan_esc_max: 15
scan_timeout_ms: 200
action_sequence_file: demo_sequence.txt
```

`esc_id` / `mst_id` 由扫描自动发现，无需手写。

可选 `action_sequence_file` 指向动作序列文本（相对本 yaml 所在目录或绝对路径）。示例见 `config/demo_sequence.txt`：

```text
M1 pos=0.0 ve=1.0
delay 500
M1 pos=1.0 ve=2.0
```

`delay` 只占用时间轴，不等待电机到位；延时结束后若上一电机仍在运动，会直接执行下一步（可能覆盖同轴目标）。

## 菜单

| 编号 | 功能 |
|------|------|
| 1 | 选择电机 |
| 2 | 查询全部电机状态 |
| 3 | 使能全部电机 |
| 4 | 失能全部电机 |
| 5 | 驱动选中电机（须已全部使能，且选中轴为位置速度模式） |
| 6 | 清错（当前选中电机，须已失能） |
| 7 | 重新扫描（须已失能） |
| 8 | 修改选中电机控制模式（须已失能；不写 Flash） |
| 9 | 保存参数到 Flash（当前选中电机，须已失能） |
| 10 | 执行动作序列（须已全部使能；可用配置路径或手动输入文件） |
| 11 | 将当前选中电机的位置写为零点（须已失能；输入 `YES` 确认） |
| 0 | 退出 |

使能后工具以 100 Hz 批量重发全部轴目标；未选中轴保持当前位置。发送、反馈或电机故障会停止周期发送，
但不会自动失能。退出前请显式执行「失能全部电机」。

非位置速度模式的电机同样会注册到总线，可查询状态、清错与切换模式，但不能使能或驱动。
菜单 8 可在运行时切换已注册轴的控制模式；切到非位置速度模式后须全部轴回到位置速度模式方可使能。
模式变更默认只写入 RAM，断电后恢复 Flash 原值；需持久化时用菜单 9 保存当前选中轴参数。

菜单 11 使用电机保存零点命令，与菜单 9 的参数保存不同。写入前应将机械臂固定在目标零位；
写入后电机坐标基准会改变，请重新核对机械标定和机器人配置中的 `zero_offset_motor_output_rad`。
若返回错误，先读取状态确认零点是否已改变，不要直接重复写入。
此命令与 `zayv2_bringup` 滑块 GUI 的“回零”按钮无关；后者只是向轨迹控制器发送关节 0 rad 目标。
