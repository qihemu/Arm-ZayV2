#pragma once
#include <damiao_core/h55/protocol.hpp>
#include <damiao_core/transport.hpp>
#include <condition_variable>
#include <mutex>
#include <memory>

namespace damiao::h55
{
// [H55专用] 外部仅一个TX线程调用管理/发送，一个RX线程调用receive_once。
// 不创建全局总线或隐藏周期线程；锁不跨越发送或等待反馈。
class H55Bus
{
  public:
    H55Bus(BusConfig config, std::unique_ptr<ICanTransport> transport);
    Status open(const TransportConfig &config);
    void close(); // 外部先终止/join接收线程，不隐式保证电机停车。
    Status receive_once(Deadline deadline);
    void route(const CanFrame &frame);
    std::array<MotorState, 2> snapshot() const;
    Result<RegisterValue> read_register(std::size_t motor, std::uint8_t rid);
    Status verify_configuration();
    Status enable_pair();
    Status disable_pair();
    Status poll_disabled_pair(); // 仅发送FD刷新；确认与超时由调用方状态机处理。
    Status clear_pair();
    Status send_velocity_pair(const std::array<double, 2> &speed);
    Status stop_pair(); // 故障状态也可发送已核验地址的停止动作。
    bool identities_verified() const;

  private:
    Status transmit(const Result<CanFrame> &frame);
    Status special_pair(ManagementCommand command, std::uint8_t expected);
    Status send_zero_unchecked();
    BusConfig config_;
    std::unique_ptr<ICanTransport> transport_;
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    std::array<MotorState, 2> state_{};
    CanFrame reply_{};
    bool pending_ = false;
    std::size_t pending_motor_ = 0;
    std::uint8_t pending_rid_ = 0;
    Deadline pending_since_{};
    bool verified_ = false;
    bool protection_verified_ = false;
};
} // namespace damiao::h55
