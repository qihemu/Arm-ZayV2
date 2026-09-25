#pragma once
#include <damiao_core/h55/bus.hpp>

namespace robot_wheel_control
{
// 轮执行后端边界：上层提交双轮速度/启停并读取测量，不依赖USB或MCU帧格式。
// 新增MCU时实现本接口；原H55驱动和MCU固件不能同时拥有同一电机总线。
class IWheelBackend
{
  public:
    virtual ~IWheelBackend() = default;
    virtual damiao::Status open() = 0;
    virtual void close() = 0;
    virtual damiao::Status receive_once(damiao::Deadline deadline) = 0;
    virtual std::array<damiao::MotorState, 2> snapshot() const = 0;
    virtual damiao::Status verify_configuration() = 0;
    virtual damiao::Status enable_pair() = 0;
    virtual damiao::Status disable_pair() = 0;
    virtual damiao::Status poll_disabled_pair() = 0;
    virtual damiao::Status clear_pair() = 0;
    virtual damiao::Status send_velocity_pair(const std::array<double, 2> &speed) = 0;
    virtual damiao::Status stop_pair() = 0;
};

// 主机直连实现复用共享damiao_core的H55模块；假传输仅编译进离线测试程序。
class DirectCanWheelBackend final : public IWheelBackend
{
  public:
    DirectCanWheelBackend(damiao::h55::BusConfig bus_config, damiao::TransportConfig transport_config,
                          std::unique_ptr<damiao::ICanTransport> transport)
        : bus_(bus_config, std::move(transport)), transport_config_(std::move(transport_config))
    {
    }
    damiao::Status open() override
    {
        return bus_.open(transport_config_);
    }
    void close() override
    {
        bus_.close();
    }
    damiao::Status receive_once(damiao::Deadline deadline) override
    {
        return bus_.receive_once(deadline);
    }
    std::array<damiao::MotorState, 2> snapshot() const override
    {
        return bus_.snapshot();
    }
    damiao::Status verify_configuration() override
    {
        return bus_.verify_configuration();
    }
    damiao::Status enable_pair() override
    {
        return bus_.enable_pair();
    }
    damiao::Status disable_pair() override
    {
        return bus_.disable_pair();
    }
    damiao::Status clear_pair() override
    {
        return bus_.clear_pair();
    }
    damiao::Status poll_disabled_pair() override
    {
        return bus_.poll_disabled_pair();
    }
    damiao::Status send_velocity_pair(const std::array<double, 2> &speed) override
    {
        return bus_.send_velocity_pair(speed);
    }
    damiao::Status stop_pair() override
    {
        return bus_.stop_pair();
    }

  private:
    damiao::h55::H55Bus bus_;
    damiao::TransportConfig transport_config_;
};
} // namespace robot_wheel_control
