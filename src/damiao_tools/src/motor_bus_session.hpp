#pragma once

#include "config.hpp"
#include "motor_scanner.hpp"

#include <damiao_core/bus.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace damiao_tools
{

// 单 CAN 总线上多台可操作电机的注册、全使能与批量控制会话。
class MotorBusSession
{
public:
    MotorBusSession(const ToolConfig& config, const std::vector<DiscoveredMotor>& motors,
        std::unique_ptr<damiao::ICanTransport> transport = std::make_unique<damiao::SocketCanTransport>());
    ~MotorBusSession();
    MotorBusSession(const MotorBusSession&) = delete;
    MotorBusSession& operator=(const MotorBusSession&) = delete;

    damiao::Status initialize();
    damiao::Result<damiao::MotorState> status(damiao::MotorIndex index);
    damiao::Status enable_all();
    damiao::Status disable_all();
    damiao::Status drive(damiao::MotorIndex index, double absolute_position_rad, double speed_rad_s);
    damiao::Status clear_error(damiao::MotorIndex index);
    damiao::Status shutdown();

    std::size_t motor_count() const noexcept;
    bool all_enabled() const noexcept;
    bool control_active() const noexcept;
    damiao::ErrorCode background_error() const noexcept;
    damiao::BusState bus_state() const;
    const DiscoveredMotor& motor_info(damiao::MotorIndex index) const;

private:
    void control_loop();
    damiao::Status stop_control_loop();
    damiao::Deadline operation_deadline() const;

    ToolConfig config_;
    std::vector<DiscoveredMotor> motors_;
    damiao::DamiaoBus bus_;
    std::vector<damiao::PositionVelocityCommand> commands_;
    mutable std::mutex command_mutex_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> all_enabled_{false};
    std::atomic<bool> control_active_{false};
    std::atomic<bool> stop_requested_{true};
    std::atomic<damiao::ErrorCode> background_error_{damiao::ErrorCode::Ok};
    std::thread control_thread_;
};

}  // namespace damiao_tools
