#pragma once

#include <damiao_core/types.hpp>

namespace damiao
{

// 本版只支持经典 CAN 标准帧，不配置接口波特率或启停网卡。
struct TransportConfig
{
    std::string interface_name = "can0";
    bool passive = false;
    // 所有协作程序必须使用相同目录；默认 /tmp 不要求修改系统目录权限。
    std::string lock_directory = "/tmp";
};

// 允许一个接收者与发送者并发；发送者之间以及 open/close 须外部串行化。
// close 前必须结束接收线程，不提供并发 close/receive 保证。
// send 为非阻塞接口，成功仅代表主机接受帧，不能证明电机执行。
// receive 使用 steady_clock 绝对截止时间，超时应返回 Timeout。
class ICanTransport
{
public:
    // 允许通过接口指针安全释放具体传输后端。
    virtual ~ICanTransport() = default;

    // 打开并配置传输资源，失败时保持后端为关闭状态。
    virtual Status open(const TransportConfig& config) = 0;
    // 非阻塞发送一个经典 CAN 标准数据帧。
    virtual Status send(const CanFrame& frame) = 0;
    // 仅返回发送错误码；自定义后端可覆写以避免周期路径构造字符串。
    virtual ErrorCode send_code(const CanFrame& frame)
    {
        return send(frame).code;
    }
    // 接收一个数据帧，最迟等待到 steady_clock 绝对截止时间。
    virtual Result<CanFrame> receive(Deadline deadline) = 0;
    // 幂等释放传输资源，不向电机发送管理命令。
    virtual Status close() = 0;
    // 查询传输资源当前是否已经成功打开。
    virtual bool is_open() const noexcept = 0;
};

// Linux SocketCAN 后端：主动连接持有协作式进程锁，被动连接拒绝所有发送。
// 不创建线程，不自动使能/切模式/存参数；析构只释放主机资源。
class SocketCanTransport final : public ICanTransport
{
public:
    // 构造一个尚未打开且不持有系统资源的 SocketCAN 后端。
    SocketCanTransport();
    // 释放 SocketCAN 文件描述符和协作式进程锁。
    ~SocketCanTransport() override;
    // 文件描述符和进程锁具有唯一所有权，因此禁止复制。
    SocketCanTransport(const SocketCanTransport&) = delete;
    // 禁止复制赋值，避免两个实例管理同一组系统资源。
    SocketCanTransport& operator=(const SocketCanTransport&) = delete;
    // 后端地址在总线对象中保持稳定，因此禁止移动。
    SocketCanTransport(SocketCanTransport&&) = delete;
    // 禁止移动赋值，保持总线持有的后端地址和资源归属稳定。
    SocketCanTransport& operator=(SocketCanTransport&&) = delete;

    // 打开 SocketCAN 接口，并在主动模式下取得接口独占协作锁。
    Status open(const TransportConfig& config) override;
    // 校验并非阻塞发送一个经典 CAN 标准数据帧。
    Status send(const CanFrame& frame) override;
    // 执行无诊断文本分配的快速发送并返回错误码。
    ErrorCode send_code(const CanFrame& frame) override;
    // 轮询套接字直至收到一个合规数据帧或到达截止时间。
    Result<CanFrame> receive(Deadline deadline) override;
    // 幂等关闭套接字并释放锁，不隐式发送失能命令。
    Status close() override;
    // 查询 SocketCAN 套接字是否有效。
    bool is_open() const noexcept override;

private:
    int socket_fd_ = -1;
    int lock_fd_ = -1;
    bool passive_ = false;
};

}  // namespace damiao
