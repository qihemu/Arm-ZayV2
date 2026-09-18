#pragma once

#include "config.hpp"
#include "motor_bus_session.hpp"
#include "motor_scanner.hpp"

#include <damiao_core/transport.hpp>

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

namespace damiao_tools
{

// 编排扫描结果、全局 M 编号与单总线多电机会话。
class MotorManager
{
public:
    MotorManager() = default;

    damiao::Status scan_and_initialize(const ToolConfig& config,
        std::unique_ptr<damiao::ICanTransport> transport = std::make_unique<damiao::SocketCanTransport>());
    damiao::Status rescan(const ToolConfig& config,
        std::unique_ptr<damiao::ICanTransport> transport = std::make_unique<damiao::SocketCanTransport>());

    const std::vector<DiscoveredMotor>& motors() const noexcept;
    const ToolConfig& config() const noexcept;
    std::size_t selected_list_index() const noexcept;
    std::size_t operable_count() const noexcept;
    std::size_t registered_count() const noexcept;
    bool all_enabled() const noexcept;
    bool control_active() const noexcept;
    damiao::ErrorCode background_error() const noexcept;
    damiao::BusState bus_state() const;

    damiao::Status select_motor(std::size_t list_index_one_based);
    damiao::Result<damiao::MotorState> status_selected();
    std::vector<damiao::Result<damiao::MotorState>> status_all();
    damiao::Status enable_all();
    damiao::Status disable_all();
    damiao::Status drive_selected(double absolute_position_rad, double speed_rad_s);
    damiao::Status clear_error_selected();
    damiao::Status shutdown();

private:
    std::optional<damiao::MotorIndex> session_index_for_list(std::size_t list_index) const;
    damiao::Status rebuild_session(std::unique_ptr<damiao::ICanTransport> transport);

    ToolConfig config_{};
    std::vector<DiscoveredMotor> motors_;
    std::vector<std::optional<damiao::MotorIndex>> list_to_session_;
    std::unique_ptr<MotorBusSession> session_;
    std::size_t selected_list_index_ = 0;
};

}  // namespace damiao_tools
