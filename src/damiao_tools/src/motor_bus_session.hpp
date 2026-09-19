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
    // 读回 CTRL_MODE；维护态且已失能时访问总线寄存器。
    damiao::Result<damiao::ControlMode> read_control_mode(damiao::MotorIndex index);
    // 写入 CTRL_MODE 并读回确认；写前刷新全部轴失能反馈。
    damiao::Status set_control_mode(damiao::MotorIndex index, damiao::ControlMode mode);
    // 将当前 RAM 参数写入 Flash；须已失能。
    damiao::Status save_parameters(damiao::MotorIndex index);
    damiao::Status shutdown();

    std::size_t motor_count() const noexcept;
    bool all_enabled() const noexcept;
    bool control_active() const noexcept;
    damiao::ErrorCode background_error() const noexcept;
    damiao::BusState bus_state() const;
    const DiscoveredMotor& motor_info(damiao::MotorIndex index) const;
    // 从总线缓存刷新会话内电机状态，供界面展示。
    void refresh_motor_cache();

private:
    void control_loop();
    damiao::Status stop_control_loop();
    damiao::Deadline operation_deadline() const;
    // 主动查询全部轴并确认失能，刷新 damiao_core 管理门控所需反馈。
    damiao::Status ensure_all_motors_disabled();
    // Flash 写入后等待目标轴恢复并重新同步配置。
    damiao::Status resync_motor_after_save(damiao::MotorIndex index);

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
