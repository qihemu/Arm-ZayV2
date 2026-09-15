# damiao_core

`damiao_core` 是不依赖 ROS 的 C++17 动态库骨架，依据项目
`docs/damiao-development-guide.md` 建立协议层和传输层接口。

## 当前状态

- 可独立编译、安装，并通过 `damiao_core::damiao_core` 链接。
- `DamiaoProtocol` 的所有编解码方法返回 `Unsupported`，结果值为空。
- `SocketCanTransport::open/send/receive` 返回 `Unsupported`；`open` 不创建 Socket，
  `is_open()` 始终为 false，`close()` 幂等返回 `Ok`。
- 构造和析构不访问硬件。没有接收线程或周期发送线程，不会自动使能、切换模式、
  保存参数、清错或发送运动命令。
- `can0` 仅为配置默认值。接口限定经典 CAN 标准数据帧，未实现 CAN FD，
  不配置波特率、不启停网卡。本轮未进行实机查询或运动验收。
- 尚未加入多电机总线、参数事务、所有权、CLI、GUI 或 ROS 硬件插件。

## 独立构建与安装

在工作区根目录执行，无需加载 ROS 环境：

```bash
./scripts/build_damiao_core.sh
# 可选：向 CMake 传递配置参数。
./scripts/build_damiao_core.sh -DCMAKE_BUILD_TYPE=Debug
```

脚本默认使用 Release，构建目录为工作区的 `build/damiao_core`，不执行安装。
也可手动指定构建和安装目录：

```bash
cmake -S src/damiao_core -B /tmp/damiao_core-build \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/tmp/damiao_core-install
cmake --build /tmp/damiao_core-build --parallel
cmake --install /tmp/damiao_core-build
```

安装包含 `include/damiao_core`、`libdamiao_core.so.0.1.0` 及版本链接、
CMake Config/Version/Targets，以及包元数据和本说明。
库目录由 `GNUInstallDirs` 决定；包版本为 `0.1.0`，SOVERSION 为 `0`。
当前接口仍处于设计阶段，不承诺跨编译器或后续接口修改的 C++ ABI 兼容。

## 消费者链接

独立消费者的 `CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.16)
project(damiao_consumer LANGUAGES CXX)

find_package(damiao_core CONFIG REQUIRED)
add_executable(damiao_consumer main.cpp)
target_link_libraries(damiao_consumer PRIVATE damiao_core::damiao_core)
```

`main.cpp` 可通过以下调用识别当前占位状态，不访问硬件：

```cpp
#include <damiao_core/transport.hpp>

int main()
{
    // 骨架尚不建立连接，调用者必须检查错误码。
    damiao::SocketCanTransport transport;
    const auto status = transport.open(damiao::TransportConfig{});
    return status.code == damiao::ErrorCode::Unsupported ? 0 : 1;
}
```

配置消费者时传入 `-DCMAKE_PREFIX_PATH=/tmp/damiao_core-install`。
导出 target 自动提供头文件路径、动态库和 C++17 要求。

## colcon 接入

`package.xml` 的构建类型为 `cmake`，只提供发现元数据，不依赖 ament：

```bash
colcon list --base-paths src --packages-select damiao_core
colcon build --packages-select damiao_core
```

本轮检查范围为独立构建、安装后外部消费者链接和 colcon 包发现；
未添加协议行为测试。

## 接口约定与后续实现

- 公共头文件为 `types.hpp`、`protocol.hpp`、`transport.hpp`。
  CAN 帧类型在公共类型层定义，协议层不依赖传输对象或 Linux 头文件。
- 电机状态和位置速度命令使用减速器输出轴的 rad、rad/s、N·m；
  报告力矩是电机估计量。`MappingLimits` 为逐电机映射范围，不能当作机械限位。
- `MotorState::valid` 区分有效反馈和默认值，接收时间采用 `steady_clock`；
  该时间不代表设备采样时刻。
- 协议接口无状态，可并发调用。同一传输实例的所有操作须由调用者串行化；
  后续接收线程由总线层管理。本骨架不承诺硬实时或无内存分配。
- 后续 `send` 为非阻塞操作，成功只代表主机接受帧；`receive` 使用绝对截止时间。
  当前占位方法不等待截止时间，立即返回 `Unsupported`。
- 后续编解码需验证标准帧、长度、地址、浮点有限性和范围；
  扩展帧、RTR、错误帧和 CAN FD 不能交给普通反馈解码。
- 寄存器号以 `uint8_t` 传递，值采用 `variant<float, uint32_t>`。
  寄存器表与类型校验尚未实现，后续以项目内新版电机手册为准，
  不照搬旧例程的寄存器类型判断。
- 寄存器回应接口提供期望 MST_ID、ESC_ID、操作码与 RID，
  仅预留字段匹配边界，不宣称解决迟到回应或数据重合歧义。
- 通用管理接口只预留帧编码；运行许可、失能前置条件、写入读回及
  存参数完成确认留给后续管理层。
