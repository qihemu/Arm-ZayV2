#pragma once

#include <damiao_core/types.hpp>

namespace damiao
{

// 本版只支持经典 CAN 标准帧，不配置接口波特率或启停网卡。
struct TransportConfig
{
    std::string interface_name = "can0";
};

// 调用者串行化同一实例的全部操作；不提供并发 close/receive 保证。
// send 为非阻塞接口，成功仅代表主机接受帧，不能证明电机执行。
// receive 使用 steady_clock 绝对截止时间，超时应返回 Timeout。
// 实际收发与期限处理尚未实现，当前收发立即返回 Unsupported。
class ICanTransport
{
public:
    virtual ~ICanTransport() = default;

    virtual Status open(const TransportConfig& config) = 0;
    virtual Status send(const CanFrame& frame) = 0;
    virtual Result<CanFrame> receive(Deadline deadline) = 0;
    virtual Status close() = 0;
    virtual bool is_open() const noexcept = 0;
};

// SocketCAN 占位后端：不持有 Socket，不启线程，构造和析构无硬件动作。
class SocketCanTransport final : public ICanTransport
{
public:
    SocketCanTransport();
    ~SocketCanTransport() override;
    SocketCanTransport(const SocketCanTransport&) = delete;
    SocketCanTransport& operator=(const SocketCanTransport&) = delete;
    SocketCanTransport(SocketCanTransport&&) = delete;
    SocketCanTransport& operator=(SocketCanTransport&&) = delete;

    Status open(const TransportConfig& config) override;
    Status send(const CanFrame& frame) override;
    Result<CanFrame> receive(Deadline deadline) override;
    // 骨架没有资源需要释放，close 幂等返回 Ok；is_open 始终为 false。
    Status close() override;
    bool is_open() const noexcept override;
};

}  // namespace damiao
