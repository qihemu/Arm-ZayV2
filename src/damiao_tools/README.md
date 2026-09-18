# damiao_tools

基于 `damiao_core` 的单 CAN 总线多电机交互式工具。启动后自动扫描配置接口上的电机，
注册全部可操作轴（位置速度模式、最多 6 台），并提供数字菜单进行状态查询、全使能、
驱动选中轴与显式失能。

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
```

`esc_id` / `mst_id` 由扫描自动发现，无需手写。

## 菜单

| 编号 | 功能 |
|------|------|
| 1 | 选择电机 |
| 2 | 查询当前选中电机状态 |
| 3 | 查询全部电机状态 |
| 4 | 使能全部电机 |
| 5 | 失能全部电机 |
| 6 | 驱动选中电机（须已全部使能） |
| 7 | 清错（当前选中电机，须已失能） |
| 8 | 重新扫描（须已失能） |
| 0 | 退出 |

使能后工具以 100 Hz 批量重发全部轴目标；未选中轴保持当前位置。发送、反馈或电机故障会停止周期发送，
但不会自动失能。退出前请显式执行「失能全部电机」。

非位置速度模式的电机会在列表中展示，但不会注册，也不能驱动。
