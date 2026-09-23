# Arm-ZayV2

## ZayV2 模型与 MoveIt

```bash
colcon build --packages-up-to arm_control
source install/setup.bash
ros2 launch arm_control arm_control.launch.py
```

如需使用 Servo 键盘演示，运行 `ros2 launch arm_control servo_demo.launch.py`，
再在交互终端运行 `ros2 launch arm_control servo_keyboard.launch.py`。
当前 `zayv2_moveit_config` 使用 `mock_components/GenericSystem` 模拟硬件；
真机接入需要单独实现并配置达妙硬件接口。

达妙电机核心库位于 `src/damiao_core`，最小单电机链路测试工具位于
`src/damiao_tools`。工具启动后通过阻塞式提示符接收状态、使能、绝对位置驱动和
显式失能命令：

```bash
colcon build --packages-up-to damiao_tools
source install/setup.bash
damiao_motor_tool --file src/damiao_tools/config/motor.example.yaml
```

示例配置中的运动限制为空，不能直接用于真机运动。工具不会切换模式、清错或在退出时
自动失能；完整边界和命令见 `src/damiao_tools/README.md`。
