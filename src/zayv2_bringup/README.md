# 达妙硬件 bringup

本包保留 DEV-07 单轴台架入口，并新增六轴 `damiao_six_axis.launch.py`。六轴入口加载实际 ZayV2 机器人几何，为 `joint1`～`joint6` 注册一套 `DamiaoArm` 硬件插件和 `arm_controller`。它与六轴 Mock 演示分开运行，不能同时启动两个 controller_manager 控制同一硬件。

## 六轴配置与启动

```bash
source /opt/ros/humble/setup.bash
cd /home/wlzc/qihemu_ws/Arm-ZayV2
colcon build --packages-up-to zayv2_bringup damiao_hardware
source install/setup.bash
cp src/zayv2_bringup/config/six_axis.example.yaml /tmp/damiao_six_axis.yaml
# 按设备读回、机械标定和受限台架记录填写所有 null 字段。
ros2 launch zayv2_bringup damiao_six_axis.launch.py config_file:=/tmp/damiao_six_axis.yaml
```

六轴 YAML 是当前 bringup 的唯一电机地址、方向、零偏和限位来源。启动前检查六个关节的顺序、ID 重复及串轴冲突、有限数和期限关系，并要求关节范围不超过当前 ZayV2 URDF 模型。接口必须是已存在的 SocketCAN 网卡。模板中的 `null` 会阻止启动；本包不会设置网卡位率。

`DamiaoArm` 在 controller_manager 中明确从 **inactive** 开始；状态广播器启动，六轴 `arm_controller` 只加载为 **inactive**。配置阶段会对六台电机发送管理读请求和无运动状态查询，但不会发送位置控制帧或自动使能。`allow_enable_on_activate` 默认 `false`。

在全部六轴的反馈、方向、零位、限位、设备 TIMEOUT、机械支撑及停止行为均完成台架验证后，才可在独立配置副本中明确允许使能，并按顺序执行：

```bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 control set_hardware_component_state DamiaoArm active
ros2 control set_controller_state arm_controller active
```

停止轨迹并确认机械支撑后，先将 `arm_controller` 设为 inactive，再将 `DamiaoArm` 设为 inactive。插件会撤销整组发送许可并逐台失能；这些操作不是急停。六轴帧在同一控制周期内由核心库批量提交，但 CAN 总线上仍逐帧发送。实机同步精度、故障联动和停止距离需要测量。

## 单轴台架入口

`damiao_single_axis.launch.py` 使用独立的 `single_axis.example.yaml`，仅包含测试关节 `joint1` 和测试连杆。控制器名为 `single_axis_controller`。启动方式相同，只需将 launch 文件和配置模板换为单轴版本；该模型不代表整台机械臂几何。单轴与六轴配置均默认禁止使能。
