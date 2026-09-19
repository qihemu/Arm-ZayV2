# damiao_core

不依赖 ROS 的 Linux/C++17 达妙电机核心库，依据项目
`docs/damiao-development-guide.md` 实现，导出 `damiao_core::damiao_core`。

## 当前能力

- **协议层**：位置速度帧、普通反馈、类型化寄存器读写、独立状态查询、
  使能/失能/清错/保存零点帧及 Flash 保存请求与回应。
  float 按 IEEE-754 小端处理，拒绝非法地址、长度、NaN/Inf、溢出、
  非零数下溢为零、负速度上限、寄存器类型错误、只读与未知寄存器写入。
- **寄存器表**：使用本项目 J4310P V1.1 第 16～17 页、J4340 V1.2 第 17～18 页。
  新版 `0x37 dir` 是 float，`0x38` 是 `m_off`；不复制旧例程的类型猜测。
  `OV_Value` 的范围为 TBD，写入返回 `Unsupported`。
- **SocketCAN**：绑定已有接口，默认 `can0`，仅发送经典 CAN 标准数据帧。
  非阻塞发送、绝对截止时间接收、检查 `ssize_t`/`CAN_MTU`/有效长度与标志，
  分别处理 EINTR、EAGAIN、缓冲区满、断连和 CAN 错误。
- **控制权**：主动 Socket 持有 `flock` 协作式锁；锁名按网络命名空间和 ifindex
  规范化，包含持有者 PID。被动 Socket 可共存，但拒绝所有发送，包括参数查询。
- **总线层**：最多六台电机，运行前注册，固定索引与状态存储；按完整 MST_ID
  路由后核对反馈低位 ESC_ID，线程安全快照与每轴新鲜度检查。
  地址重复、名称重复、低八位反馈/控制冲突及管理 ID 冲突均拒绝。
- **维护操作**：一条总线同一时刻只有一个管理操作；并发请求返回 `WouldBlock`。
  维护事务使用同一绝对截止时间、静默间隔与完整匹配字段。
  参数写入执行读原值、写入、读回比较，不隐式存 Flash。
  写报告保留原值、请求值、读回值、是否发出与是否验证成功。
- **运行门控**：`Closed / Maintenance / Control / Fault`。
  连接不查询、不使能、不切模式、不存参数。
  全轴状态新鲜、使能、位置速度模式、地址/映射/模式读回且运动限位完整后，
  调用者才可显式打开批量发送许可。
  批量命令先全量校验、再逐帧非阻塞发送，返回预分配的逐帧错误码；
  未发送帧标记 `NotExecuted`，中途失败标记 `PartialFailure` 并锁存故障。
- **诊断**：接收/拒绝/歧义帧、事务超时、发送失败计数，最后错误码和传输错误详情。

## 构建、测试和安装

在工作区根目录执行，无需加载 ROS：

```bash
./scripts/build_damiao_core.sh
ctest --test-dir build/damiao_core --output-on-failure
```

脚本默认 Release，产物在 `build/damiao_core`，只配置和编译，不自动运行测试或安装。
可传入 `-DCMAKE_BUILD_TYPE=Debug`；只构建库时传入 `-DBUILD_TESTING=OFF`。
独立构建与安装也可指定自己的目录：

```bash
cmake -S src/damiao_core -B /tmp/damiao_core-build \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/tmp/damiao_core-install
cmake --build /tmp/damiao_core-build --parallel
ctest --test-dir /tmp/damiao_core-build --output-on-failure
cmake --install /tmp/damiao_core-build
```

安装包含公共头文件、`libdamiao_core.so.0.1.0` 及版本链接、CMake Config/Version/Targets、
包元数据和本说明；目录由 `GNUInstallDirs` 决定。版本 `0.1.0`，SOVERSION `0`。
当前接口仍处于开发阶段，不承诺跨编译器或后续接口变更的 C++ ABI 兼容。
`package.xml` 的构建类型为 `cmake`，不依赖 ament：

```bash
colcon list --base-paths src --packages-select damiao_core
colcon build --packages-select damiao_core
```

## 消费者链接

```cmake
cmake_minimum_required(VERSION 3.16)
project(damiao_consumer LANGUAGES CXX)

find_package(damiao_core CONFIG REQUIRED)
add_executable(damiao_consumer main.cpp)
target_link_libraries(damiao_consumer PRIVATE damiao_core::damiao_core)
```

以下 `main.cpp` 仅演示纯协议编码，不连接或发送报文：

```cpp
#include <damiao_core/protocol.hpp>

int main()
{
    // 检查状态码和可选结果，再决定是否使用编码帧。
    const auto result = damiao::DamiaoProtocol::encode_position_velocity(1, {1.0, 2.0});
    return result.status.code == damiao::ErrorCode::Ok && result.value ? 0 : 1;
}
```

配置消费者时传入 `-DCMAKE_PREFIX_PATH=/tmp/damiao_core-install`。
导出 target 自动提供头文件路径、动态库和 C++17 要求。

## 总线接口与责任边界

公共头文件为 `types.hpp`、`registers.hpp`、`protocol.hpp`、`transport.hpp`、`bus.hpp`。
核心接受已经解析的 C++ 配置，不读取 YAML，也不调用 ROS。

- 注册通过 `register_motor` 完成，返回稳定索引。只有关闭状态能注册，最多六轴。
  `BusConfig` 必须提供正值 `feedback_timeout` 与 `management_quiet_period`，
  不采用未经实测的电机保护期限。
- `open` 只获取发送权和建立接收线程；主动连接清除映射/模式可信标记和反馈有效性。
  `synchronize_motor` 逐项读取 ESC_ID、MST_ID、模式、PMAX/VMAX/TMAX 与软件版本，
  全部通过后发布该轴配置。`query_state` 使用独立 `0xCC` 查询，不发送零位置目标。
- 参数读取允许启动时尚未获得反馈，但使用方须确保是已停止的维护会话。
  任一已知使能轴会阻止参数事务；写入与存参数还要求全轴新鲜反馈确认失能。
  状态查询不与 Control 状态交错，周期状态使用接收缓存。
- 类型化配置接口（维护态）：`read_control_mode`、`read_mapping_limits`、
  `read_communication_timeout`；`set_control_mode`（含 MIT 模式写入）、
  `write_mapping_limits`、`write_communication_timeout`（50μs/计数换算）、
  `save_zero_position`（`save_zero` 别名）。通用寄存器仍用
  `read_parameter` / `write_parameter_verified`；寄存器 ID 见 `RegisterId` 命名空间。
- `enable`、`disable`、`clear_error`、`save_zero` 均为显式低频操作，
  通过后续独立状态查询确认相应状态；总线首版只开放位置速度模式的使能。
  **上层仍必须完成已验证的首次目标保持、使能顺序与停止/支撑策略。**
  本库不会猜测失能期间目标是否保留，也不保证单独调用 enable 不产生跳变。
- `begin_control` 只开放发送许可，不发送目标、不使能；`end_control` 只撤销许可，
  不意味着已停车，不会隐式失能承重轴。Control 状态拒绝直接 close。
  显式 disable 撤销整组新目标许可；是否应失能由上层停止策略决定。
- 运动参数是电机减速器输出轴的 rad、rad/s，速度是最大绝对速度。
  `MotorConfig` 的最小/最大输出轴位置与最大速度必须补齐，不能用映射范围替代。
  关节转换、标定、加速度/变化率、任务目标有效期和异常循环周期由插件/调试会话负责。
- `snapshot` 在状态码为故障或过期时仍可返回诊断样本，使用方必须检查 status；
  `snapshot_into` 复制到预分配数组并将不可使用样本的 valid 置为 false。
  每轴 sequence 只在接受新时间戳反馈时递增，不刷新旧样本时间。
  诊断样本不能用于恢复控制；故障恢复必须检查总线状态并显式重新连接/同步。
- 写入、模式或零位变更使旧反馈失效并增加本地配置版本；
  配置提交记录生效时间，拒绝用新配置解码此前接收的旧队列反馈。
  映射写入只更新目标电机；模式切换之后必须重新建立上层保持目标。
  保存零点会改变坐标基准，外部机械标定必须重新验证。
- 超时或歧义在已发出事务后锁存 Fault；未发送的静默窗口超时不会声称设备已执行。
  不自动重试写操作、不自动回滚、不自动重连、不排队补发旧命令。
  结果未知时应在显式恢复会话中重新读回评估，而不是再次盲写。
- `send_code` 是 SocketCAN 的无字符串、无队列周期发送路径。
  批量发送和 `snapshot_into` 用 try-lock，竞争返回 `WouldBlock`；
  自定义传输应遵守并发接收/发送和非阻塞发送契约，必要时覆写 send_code。
  短临界区与 Linux 调度仍须测量，不能据软件测试宣称硬实时。
- 接收线程采用 10ms 限时轮询，资源退出先 join 再关 Socket。
  close 遇到正在执行的管理操作返回 WouldBlock；销毁对象前须结束全部外部调用。
  析构只回收主机资源，不发送物理停车或失能帧。

## 仍需验证或扩展

- 相邻 `damiao_tools` 包只提供最小单电机注册、状态、使能、位置速度驱动和显式失能
  链路测试；完整调试会话、GUI、ROS 硬件插件、多轴运动以及 MIT、速度、力位混控
  运动接口尚未开放。
- 直接协议编码可生成合法 ID/经典 CAN 波特率参数帧，但总线层的在线 ID/波特率迁移
  返回 `Unsupported`；需验证新地址回应、读回和切换时序后再开放。
- 寄存器协议没有通用事务序号。匹配字段、维护窗口和静默隔离不能消除全部
  迟到回应与普通反馈内容重合；无匹配上下文的参数形状帧保守丢弃并计入歧义计数。
  已观测的完全重合状态查询返回 `AmbiguousReply`，不能宣称所有回应已无歧义。
- `0xCC` 支持、位置反馈与 p_m/xout 的关系、模式切换、首次使能、TIMEOUT 保护、
  Flash 与零点行为均仍需实体固件验证。本轮不运行任何实机查询或运动命令。
- 内核接收时间戳为 realtime，本库按当前时钟差转换为 steady 时间，保留 Socket 队列年龄。
  缺失/截断/明显未来时间戳拒绝。系统时钟跳变仍会影响年龄估算，不能当作设备采样时刻。
- 协作锁默认放在 `/tmp`；插件、CLI 和其他本库程序必须使用相同目录约定。
  跨用户部署应预先配置共享目录与权限。锁不能阻止未遵守约定的第三方工具或其他物理主站。

## 软件测试与可选 vcan

默认软件用例使用独立已知向量、内存模拟电机和临时锁文件，不创建 CAN Socket。
覆盖协议/寄存器边界、原始帧标志与 MTU、多进程所有权、注册与路由、维护写入证据、
映射/模式/零点失效、整组预检、部分发送失败、被动监听、过期、乱序/重复、
管理竞争、迟到与歧义回应以及断连。模拟器不证明实际电机保护与停止行为。

2026-09-16 软件验证记录：GCC 11.4.0，Release 严格警告编译通过，
11 组软件用例通过；ASan/UBSan/LeakSanitizer 检查通过；
安装后外部消费者发现、链接与运行及 colcon 包发现通过。vcan 本轮跳过。

vcan 测试默认跳过；只有显式提供一个已有、启用的 `vcan*` 接口时才运行：

```bash
DAMIAO_CORE_VCAN_INTERFACE=vcan0 ctest --test-dir build/damiao_core \
    -R '^damiao_core_vcan$' --output-on-failure
```

该测试检查内核收发、接收时间戳、被动发送门控、期限与协作锁，
不会自动创建网卡，也不自动选择 `can0`。
