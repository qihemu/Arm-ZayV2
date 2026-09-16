# damiao_tools

基于 `damiao_core` 的最小单电机交互式链路测试工具。它只支持注册一台已配置为
位置速度模式的电机、查询状态、使能、发送绝对输出轴位置目标以及显式失能。

## 构建

```bash
colcon build --packages-up-to damiao_tools
source install/setup.bash
```

工具依赖系统 `yaml-cpp`。示例配置中的运动限制是空值，必须按台架实际情况填写：

```bash
damiao_motor_tool --file src/damiao_tools/config/motor.example.yaml
```

## 命令

```text
status
enable
drive <absolute_position_rad> <speed_rad_s>
disable
help
quit
```

使能后工具以 100 Hz 重发当前位置或最新目标。发送、反馈或电机故障会停止周期发送，
但不会自动失能。`quit`、Ctrl-C 和输入 EOF 同样只停止主机发送并关闭 SocketCAN；
若要失能，必须在退出前明确执行 `disable`。

本工具不切换控制模式、不清错、不保存零点或参数，也不提供机械支撑与运动轨迹插值。
