# Arm-ZayV2

## ZayV2 模型与 MoveIt

```bash
colcon build --packages-up-to arm_control
source install/setup.bash
ros2 launch arm_control arm_control.launch.py
```

如需使用 Servo 键盘演示，运行 `ros2 launch arm_control servo_demo.launch.py`，
再在交互终端运行 `ros2 launch arm_control servo_keyboard.launch.py`。
`zayv2_moveit_config` 保留 `mock_components/GenericSystem` 演示链路；真机
`ros2_control` 插件、单轴/六轴 bringup 已位于 `damiao_hardware` 和
`zayv2_bringup`。真机配置、启动顺序和限制见
[bringup 说明](src/zayv2_bringup/README.md)。软件接入完成不代表实机验收通过。

达妙电机核心库位于 `src/damiao_core`，多电机调试工具位于
`src/damiao_tools`。工具提供状态查询、受限位置驱动和显式失能，并可在全部
电机失能后对选中电机执行保存零点命令：

```bash
colcon build --packages-up-to damiao_tools
source install/setup.bash
damiao_motor_tool --file src/damiao_tools/config/motor.example.yaml
```

示例运动限制只是占位值，不能直接用于真机运动。工具退出时不会自动失能；
保存电机零点会改变电机坐标基准，之后需要重新核对机械零偏。
完整菜单和边界见 [调试工具说明](src/damiao_tools/README.md)。

当前开发状态与下一步验收顺序见
[实施步骤](docs/zayv2-implementation-steps.md) 和
[开发文档](docs/damiao-development-guide.md)。

统一编译、启动和按包查看日志，见[工作空间脚本说明](scripts/README.md)；三个入口位于工作空间根目录。
