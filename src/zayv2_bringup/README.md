# 达妙硬件 bringup

本包保留 DEV-07 单轴台架入口，并新增六轴 `damiao_six_axis.launch.py`。六轴入口加载实际 ZayV2 机器人几何，为 `joint1`～`joint6` 注册一套 `DamiaoArm` 硬件插件和 `arm_controller`。它与六轴 Mock 演示分开运行，不能同时启动两个 controller_manager 控制同一硬件。

## 六轴配置与启动

```bash
source /opt/ros/humble/setup.bash
cd /home/qihemu/qihemu_ws/Arm-ZayV2
colcon build --packages-up-to zayv2_bringup damiao_hardware
source install/setup.bash
cp src/zayv2_bringup/config/six_axis.example.yaml /tmp/damiao_six_axis.yaml
# 按设备读回和机械标定记录替换模板值，并复核各轴限位。
ros2 launch zayv2_bringup damiao_six_axis.launch.py config_file:=/home/qihemu/qihemu_ws/Arm-ZayV2/src/zayv2_bringup/config/six_axis.example.yaml
```

六轴 YAML 是当前 bringup 的唯一电机地址、方向、零偏和限位来源。启动前检查六个关节的顺序、ID 重复及串轴冲突、有限数和期限关系，并要求关节范围不超过当前 ZayV2 URDF 模型。接口必须是已存在的 SocketCAN 网卡。六轴模板包含占位数值，能通过格式校验并不代表已完成实物标定；本包不会设置网卡位率。

`DamiaoArm` 在 controller_manager 中明确从 **inactive** 开始；状态广播器启动，六轴 `arm_controller` 只加载为 **inactive**。配置阶段会对六台电机发送管理读请求和无运动状态查询，但不会发送位置控制帧或自动使能。`allow_enable_on_activate` 默认 `false`。

若启动时某关节角度超出配置限位，`DamiaoArm` 仍停留在 **inactive**，日志会报告关节名、实测角度和限位；此时 `ros2 control set_hardware_component_state DamiaoArm active` 会失败，电机不会使能。调整关节位置进入限位后可以再次请求激活。通信失败、设备未失能或反馈无效仍按配置故障处理。

在全部六轴的反馈、方向、零位、限位、设备 TIMEOUT、机械支撑及停止行为均完成台架验证后，才可在独立配置副本中明确允许使能，并按顺序执行：

```bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 control set_hardware_component_state DamiaoArm active
ros2 control set_controller_state arm_controller active
```

六轴启动包含独立的硬件状态监视器：`DamiaoArm` 激活后若退回非 active 状态，监视器会结束整个 launch，防止轨迹控制器继续使用旧反馈接受目标。轨迹控制器还要求六轴实测终点误差不超过 0.05 rad，并在终点后 1 秒内达到；这些阈值应按台架实测调整。硬件故障后需重新启动 launch，不能继续使用原 Action 服务。

停止轨迹并确认机械支撑后，先将 `arm_controller` 设为 inactive，再将 `DamiaoArm` 设为 inactive。插件会撤销整组发送许可并逐台失能；这些操作不是急停。六轴帧在同一控制周期内由核心库批量提交，但 CAN 总线上仍逐帧发送。实机同步精度、故障联动和停止距离需要测量。

## 单轴台架入口

`damiao_single_axis.launch.py` 使用独立的 `single_axis.example.yaml`，仅包含测试关节 `joint1` 和测试连杆。控制器名为 `single_axis_controller`。启动方式相同，只需将 launch 文件和配置模板换为单轴版本；该模型不代表整台机械臂几何。单轴与六轴配置均默认禁止使能。

## 六轴滑块测试 GUI

先按上述流程启动六轴 bringup 并手动将 `DamiaoArm`、`arm_controller` 设为 active。在另一终端运行独立测试节点，`--config-file` 必须与 bringup 使用同一份 YAML：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run zayv2_bringup damiao_slider_test_gui.py --config-file /tmp/damiao_six_axis.yaml
```

GUI 从配置读取六轴角度和速度上限，启动后从 `/arm_controller/controller_state` 读取实测角度初始化滑块；未取得完整新鲜反馈前不会发送目标。拖动可同时调整多轴，目标最多每 100 ms 合并为一条完整六轴轨迹发送到 `/arm_controller/joint_trajectory`。速度默认 0.1 rad/s，可调至配置上限；松手或关闭窗口后，控制器继续执行最后的目标。若硬件、控制器或反馈失效，界面停止发送并要求重新接收实测值后再操作。右上角“回零”按钮会将六轴目标一次性设为精确的 0 rad 并发送完整轨迹；仅在六轴零点均处于配置限位内、硬件和控制器正常且反馈新鲜时可用。这里的“回零”是关节运动，不会向电机写入零点，也不会修改 `zero_offset_motor_output_rad`；电机零点写入功能在 `damiao_tools` 菜单 11。此 GUI 不发布 `/joint_states`，测试时不要同时使用其他轨迹命令来源。
