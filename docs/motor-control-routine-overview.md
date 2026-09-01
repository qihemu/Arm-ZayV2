# motor-control-routine 仓库分析

> 本文档整理自对达妙官方 `motor-control-routine` 仓库的分析，用于 Arm-ZayV2 项目在选型、协议理解与底层驱动集成时的参考。
>
> 仓库路径：`../motor-control-routine/`（与 `Arm-ZayV2` 同级）

---

## 1. 仓库定位与边界

`motor-control-routine` 是达妙（Damiao）电机的**整机 / 平台级控制例程集合**，面向不同硬件平台和软件栈提供「如何把达妙电机跑起来」的参考实现。与纯 SDK 调用不同，这里更强调**控制流程、运行环境和整机集成**。

| 保留在 `motor-control-routine` | 已迁移到 `../电机SDK/` |
| --- | --- |
| ROS1 / ROS2 例程 | C# / C++ / Python / Matlab 的 USB2CAN SDK 示例 |
| STM32 裸机例程 | USB2CAN / USB2CANFD 二次开发绑定 |
| SocketCAN 例程 | |
| Orin 载板 CAN 例程 | |

**本质**：多平台电机控制 cookbook，核心协议实现（`damiao` + `socketcan`）在多个子项目中复制 / 复用，上层按平台包装成 ROS、裸机或独立 C++ 程序。

---

## 2. 整体目录结构

```
motor-control-routine/
├── README.md
├── ROS2 例程/              # ROS2 Humble + SocketCAN（最新、最轻量）
├── ROS1 例程/              # ROS Noetic + ros_control + USB2CANFD/CAN
├── SocketCan控制例程/      # 纯 Linux SocketCAN（C++ / Python）
├── orin载板can控制达妙电机例程/  # Jetson Orin 载板，无 ROS
└── stm32例程/              # STM32 裸机（F4/H7，经典 CAN / FDCAN）
```

体量上 `stm32例程/` 最大（约 79MB，含 HAL/CMSIS），其余以协议层 + 示例代码为主。

---

## 3. 核心协议层（所有例程的共性）

各平台例程共享同一套**达妙 CAN 协议抽象**，核心在 `damiao` 命名空间。

### 3.1 支持的电机型号（15 款）

定义于 `ROS2 例程/src/dmbot_serial/include/dmbot_serial/protocol/damiao.h`：

| 型号 | 型号 | 型号 |
| --- | --- | --- |
| DM3507 | DM4310 | DM4310_48V |
| DM4340 | DM4340_48V | DM6006 |
| DM6248P | DM8006 | DM8009 |
| DM10010L | DM10010 | DMH3510 |
| DMH6215 | DMS3519 | DMG6220 |

每款电机有独立的限位参数 `limit_param[Q_MAX, DQ_MAX, TAU_MAX]`，用于 MIT 模式下的浮点 ↔ 定点编码。

### 3.2 四种控制模式

| 模式 | 枚举值 | 指令内容 |
| --- | --- | --- |
| MIT | `0x000` | kp, kd, position, velocity, effort |
| 位置速度 | `0x100` | position, velocity |
| 速度 | `0x200` | velocity |
| 位置力矩 | `0x300` | position, velocity, effort |

CAN ID 编码规则：`实际 CAN ID = 电机 ESC_ID + 模式偏移量`（如 MIT 为 `id + 0x000`）。

### 3.3 关键类设计

- **`Motor`**：单电机状态（位置 / 速度 / 力矩 / 错误码）与参数寄存器缓存
- **`Motor_Control`**：总线管理器
  - 打开 SocketCAN 接口
  - 注册电机（按 `can_id` 和 `master_id` 双向索引）
  - 使能 / 失能、模式切换、下发控制帧
  - 接收回调解析反馈帧
- **`SocketCAN`**：Linux 原生 CAN/CANFD 封装
  - `PF_CAN` + `CAN_RAW_FD_FRAMES`
  - 独立接收线程（`SCHED_FIFO` 优先级 95）
  - 支持 `can_frame`（经典 CAN）和 `canfd_frame`（CANFD）

### 3.4 MIT 控制帧编码

实现于 `ROS2 例程/src/dmbot_serial/src/protocol/damiao.cpp` 的 `control_mit()`：

1. 将 kp、kd、position、velocity、effort 线性映射为定点数
2. kp 范围 `[0, 500]`，12 bit；kd 范围 `[0, 5]`，12 bit
3. position / velocity / effort 按电机型号 `limit_param` 映射
4. 打包为 8 字节 CAN 帧，`can_id = ESC_ID + MIT_MODE`

反馈帧在 `canframeCallback()` 中反向解码，更新 `Motor` 状态。

---

## 4. 各子项目详解

### 4.1 ROS2 例程（推荐作为 SocketCAN 集成参考）

**环境**：Ubuntu 22.04 + ROS2 Humble + C++20

**包结构**：单包 `dmbot_serial`

```
dmbot_serial/
├── include/dmbot_serial/protocol/   # damiao.h, socketcan.h
├── src/protocol/                    # 协议实现
├── src/test_motor_node.cpp          # 示例控制节点
└── launch/test_motor_node.launch.py
```

**架构特点**：

- **不依赖 ros_control**，比 ROS1 更轻
- 通过话题 `~/target_command`（`Float64MultiArray`）下发多电机指令
- 默认 1000 Hz 定时下发 + 1 Hz 反馈打印
- 默认 CANFD 5M（仲裁域 1M + 数据域 5M）

**数据流**：

```
ros2 topic pub → test_motor_node → Motor_Control → SocketCAN → CAN 总线 → 电机
                     ↑                                              ↓
              print_feedback ← canframeCallback ← 接收线程
```

**典型启动**：

```bash
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
ros2 launch dmbot_serial test_motor_node.launch.py control_mode:=mit
```

**适用场景**：在 ROS2 机械臂 / 移动平台上做达妙电机底层驱动，或作为 MoveIt2 的 hardware interface 参考。

### 4.2 ROS1 例程（完整 ros_control 栈）

**环境**：Ubuntu 20.04 + ROS Noetic + C++11

**两条硬件路线**：

| 路线 | 硬件 | 通信方式 |
| --- | --- | --- |
| `u2canfd/` | 达妙 USB2CANFD | libusb，需设备序列号 |
| `u2can/` | 达妙 USB2CAN | 串口 `/dev/ttyACM*` |

**u2canfd 包结构**：

```
u2canfd/src/
├── dmbot_serial/     # 协议 + USB 通信（非 SocketCAN）
├── dm_hw/            # ros_control HardwareInterface
├── dm_controllers/   # 控制器插件
├── dm_common/        # HybridJointInterface 等
└── dm_examples/      # 独立示例
```

**架构特点**：

- 标准 **ros_control** 分层：`DmHW` → `DmController` → `load_dm_hw.launch`
- `DmHW` 注册 `JointStateInterface` + `HybridJointMitInterface`
- 多电机需在 `DmHW.cpp` 和 `DmController.cpp` 中手动配置 CAN ID
- 5M 波特率多电机时建议加 120Ω 终端电阻

**与 ROS2 的差异**：

| 维度 | ROS1 | ROS2 |
| --- | --- | --- |
| 通信 | USB SDK（libusb） | SocketCAN |
| 控制框架 | ros_control | 自定义节点 |
| 配置方式 | 改 C++ 源码 | Launch 参数 |
| 复杂度 | 高（完整机器人栈） | 低（单节点演示） |

### 4.3 SocketCAN 控制例程（平台无关底层）

提供 **C++** 和 **Python** 两套等价实现，不依赖 ROS。

- **硬件**：达妙 USB2CANFD + **gs_usb SocketCAN 固件**（刷固件后无法用上位机）
- **流程**：`ip link set` 激活接口 → `cmake && make` → 运行 `test_motor`
- **Python 版**：`damiao_socketcan.py` 用标准库 `socket` 实现 CANFD，适合快速验证

**价值**：最适合理解协议本身，或集成到非 ROS 的 C++ / Python 项目。

### 4.4 Orin 载板例程（嵌入式 Linux，无 ROS）

**目标平台**：NVIDIA Jetson Orin 载板自带 CAN（`mttcan` 内核模块）

**结构**：

```
dm_hw/
├── src/main.cpp           # 信号处理 + 主循环
├── src/control_loop.cpp   # 控制循环
├── src/hardware_interface/  # damiao + socketcan + DmHW
└── CMakeLists.txt
```

**默认行为**：CAN ID `0x01`、Master ID `0x11`、MIT 模式、1M 波特率，电机按 sin 速度旋转。

**与 ROS2 的关系**：协议层几乎相同，只是封装成独立 CMake 工程，适合 Orin 上直接跑实时控制环。

### 4.5 STM32 例程（嵌入式裸机）

四个工程，按 MCU 和用途区分：

| 工程 | MCU | 特点 |
| --- | --- | --- |
| `dm_ctrl(f4) v1.1 裸机` | STM32F4 | 经典 CAN，2024-08 更新力位混控 |
| `dm_ctrl(f4)-4310_v1.0 裸机` | STM32F4 | DM4310 专用，带屏幕 + 按键交互 |
| `dm_ctrl(h7 fdcan) v1.1 裸机` | STM32H7 | FDCAN，支持 9 种波特率，≤1M 为经典 CAN，>1M 为 FDCAN |
| `dm_ctrl(DM3519 一拖四)` | STM32H7 | 一拖四电机控制 |

**代码组织**（以 F4-4310 为例）：

- `User/motor/dm4310_ctrl.c`：电机初始化、参数调节、控制循环
- `Core/Src/can.c`：CAN 外设驱动
- 通过 LCD / 按键现场调参（ID、模式、位置、速度、kp/kd）

**与 PC 端例程的差异**：直接在 MCU 上发 CAN 帧，无 SocketCAN / USB 中间层；适合作为 `ZayV2-motor-driver` 固件侧的协议参考。

---

## 5. 通信路径对比

```
┌─────────────────────────────────────────────────────────────┐
│                    达妙电机 (CAN/CANFD 总线)                  │
└─────────────────────────────────────────────────────────────┘
         ▲              ▲              ▲              ▲
         │              │              │              │
   STM32 CAN      Orin mttcan    USB2CANFD        USB2CANFD
   (裸机直连)     (SocketCAN)   (gs_usb固件)      (libusb SDK)
         │              │              │              │
    stm32例程      orin例程      ROS2/SocketCAN    ROS1 u2canfd
```

**选型建议**：

| 场景 | 推荐例程 |
| --- | --- |
| 已有 ROS2 + Linux | `ROS2 例程` |
| 需要 ros_control / MoveIt1 | `ROS1 例程` |
| 只要底层协议、无 ROS | `SocketCan控制例程` |
| Jetson Orin 载板 | `orin载板can控制达妙电机例程` |
| 电机驱动板固件 | `stm32例程` |

---

## 6. 与 Arm-ZayV2 工作区的关系

| 仓库 | 关系 |
| --- | --- |
| `Arm-ZayV2` | 机械臂整机，可将 `dmbot_serial` 封装为 ros2_control hardware interface |
| `ZayV2-motor-driver` | STM32 固件，可参考 `stm32例程` 的 CAN 协议与帧格式 |
| `moveit2` | 上层规划，底层需电机驱动提供 joint state + command interface |
| `motor-control-routine` | 达妙官方协议与多平台参考实现 |

当前 Arm-ZayV2 使用 **Mock 硬件**（`mock_components/GenericSystem`），技术方案见 [canopen-stm32-ros2-solution.md](./canopen-stm32-ros2-solution.md)。

若采用达妙电机而非自研 CANopen 从站，可参考 `ROS2 例程/dmbot_serial` 的 `Motor_Control` 与 `test_motor_node` 控制循环，封装为 `hardware_interface::SystemInterface` 与 MoveIt2 对接。

`test_motor_node` 展示了如何用 SocketCAN 驱动多电机，但**尚未对接 ros2_control**，属于底层驱动参考而非可直接接入 `arm_control.launch.py` 的成品。

---

## 7. 关键配置与注意事项

### 7.1 CAN 接口激活（SocketCAN 路线通用）

```bash
# CANFD 5M（ROS2 默认）
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on

# 经典 CAN 1M（Orin 默认）
sudo ip link set can0 up type can bitrate 1000000
```

### 7.2 电机 ID 三元组

每个电机需配置三个**等长数组**（ROS2 launch 参数）：

- `motor_types`：型号字符串，如 `DM4310`
- `motor_can_ids`：ESC ID，如 `0x01`
- `motor_master_ids`：反馈主站 ID，如 `0x11`

### 7.3 新增电机型号

需同步改三处（`dmbot_serial/README.md` 有详细说明）：

1. `damiao.h` 枚举
2. `damiao.cpp` 的 `limit_param[]`
3. `dm_ros2_utils.hpp` 的字符串映射

### 7.4 固件与工具链

- SocketCAN 路线需刷 **gs_usb 固件**（刷后不能用上位机）
- ARM 架构需额外安装 `gs_usb_drives`
- ROS1 USB 路线需配置 udev 规则（VID `34b7` / PID `6877`）

---

## 8. 代码质量与维护状态

| 方面 | 观察 |
| --- | --- |
| 文档 | 近年整理过，中英文 README + USAGE/WORKFLOW 分层清晰 |
| 代码复用 | `damiao.cpp` / `socketcan.cpp` 在多目录间复制，需注意同步 |
| ROS2 | 最活跃，C++20，结构简洁，适合二次开发 |
| ROS1 | 完整但偏重，依赖 USB SDK 而非 SocketCAN |
| STM32 | 仅 `f4-4310` 有完整 USAGE，其余以工程文件为主 |
| 历史归档 | `DMMotor_freertos.rar` 等已不再维护 |

最近提交：`fa8f511 update：电机控制例程更新`，仓库仍在维护。

---

## 9. 总结

`motor-control-routine` 的核心价值：

1. **统一的 CAN 协议实现**（`Motor` / `Motor_Control` / 四种控制模式）
2. **按平台分层的集成示例**（ROS1 完整栈 → ROS2 轻量节点 → 纯 SocketCAN → Orin 裸 C++ → STM32 固件）
3. **与 SDK 仓库的职责分离**（SDK 管 USB 调用，本仓库管整机流程）

**Arm-ZayV2 集成建议**：

- 做 **MoveIt2 + 达妙电机**：以 `ROS2 例程/dmbot_serial` 为底层驱动起点，封装 `ros2_control` `SystemInterface`
- 做 **ZayV2 电机驱动固件**：重点参考 `stm32例程` 中的 CAN 帧打包与反馈解析
- 做 **自研 CANopen 从站**：达妙协议与 CANopen CiA 402 不同，见 [canopen-stm32-ros2-solution.md](./canopen-stm32-ros2-solution.md)

---

## 10. 相关文档

| 文档 | 说明 |
| --- | --- |
| [canopen-stm32-ros2-solution.md](./canopen-stm32-ros2-solution.md) | 自研 CANopen 方案（与达妙协议路线并列） |
| [moveit2-overview.md](./moveit2-overview.md) | MoveIt2 在 Arm-ZayV2 中的角色 |
| [moveit-servo.md](./moveit-servo.md) | MoveIt Servo 实时控制 |
| `motor-control-routine/ROS2 例程/src/dmbot_serial/README.md` | 达妙官方 ROS2 例程使用说明 |
