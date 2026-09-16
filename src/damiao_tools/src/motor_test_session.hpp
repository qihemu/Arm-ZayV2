#pragma once

#include "config.hpp"

#include <damiao_core/bus.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

namespace damiao_tools
{

// 最小单电机会话：后台只重复发送最新目标，不执行插值、自动失能或故障恢复。
class MotorTestSession
{
public:
    explicit MotorTestSession(const ToolConfig& config,
        std::unique_ptr<damiao::ICanTransport> transport = std::make_unique<damiao::SocketCanTransport>());
    ~MotorTestSession();
    MotorTestSession(const MotorTestSession&) = delete;
    MotorTestSession& operator=(const MotorTestSession&) = delete;

    damiao::Status initialize();
    damiao::Result<damiao::MotorState> status();
    damiao::Status enable();
    damiao::Status drive(double absolute_position_rad, double speed_rad_s);
    damiao::Status disable();
    // 停止主机发送并关闭总线，刻意不向电机发送失能命令。
    damiao::Status shutdown();

    bool motor_enabled() const noexcept;
    bool control_active() const noexcept;
    damiao::ErrorCode background_error() const noexcept;
    damiao::BusState bus_state() const;

private:
    void control_loop();
    damiao::Status stop_control_loop();
    damiao::Deadline operation_deadline() const;

    ToolConfig config_;
    damiao::DamiaoBus bus_;
    damiao::MotorIndex motor_index_ = 0;
    mutable std::mutex command_mutex_;
    damiao::PositionVelocityCommand command_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> motor_enabled_{false};
    std::atomic<bool> control_active_{false};
    std::atomic<bool> stop_requested_{true};
    std::atomic<damiao::ErrorCode> background_error_{damiao::ErrorCode::Ok};
    std::thread control_thread_;
};

}  // namespace damiao_tools
