#include "motor_manager.hpp"

namespace damiao_tools
{
namespace
{

damiao::Status invalid_manager(const char* message)
{
    return {damiao::ErrorCode::InvalidCommand, message};
}

// 同步列表中的模式与可驱动标记；已注册轴保持 operable 不变。
void sync_discovered_mode(DiscoveredMotor& motor, damiao::ControlMode mode)
{
    motor.mode = mode;
    motor.drivable = mode == damiao::ControlMode::PositionVelocity;
    if (motor.drivable)
    {
        motor.inoperable_reason.clear();
    }
    else
    {
        motor.inoperable_reason = "非位置速度模式";
    }
}

void build_session_mapping(const std::vector<DiscoveredMotor>& motors,
    std::vector<std::optional<damiao::MotorIndex>>& mapping)
{
    mapping.assign(motors.size(), std::nullopt);
    damiao::MotorIndex session_index = 0;
    for (std::size_t list_index = 0; list_index < motors.size(); ++list_index)
    {
        if (motors[list_index].operable)
        {
            mapping[list_index] = session_index;
            ++session_index;
        }
    }
}

std::vector<DiscoveredMotor> collect_operable(const std::vector<DiscoveredMotor>& motors)
{
    std::vector<DiscoveredMotor> operable;
    for (const auto& motor : motors)
    {
        if (motor.operable)
        {
            operable.push_back(motor);
        }
    }
    return operable;
}

std::size_t first_operable_index(const std::vector<DiscoveredMotor>& motors)
{
    for (std::size_t index = 0; index < motors.size(); ++index)
    {
        if (motors[index].operable)
        {
            return index;
        }
    }
    return 0;
}

}  // namespace

damiao::Status MotorManager::rebuild_session(std::unique_ptr<damiao::ICanTransport> transport)
{
    session_.reset();
    const auto operable = collect_operable(motors_);
    if (operable.empty())
    {
        return invalid_manager("No registered motors found on the bus.");
    }
    session_ = std::make_unique<MotorBusSession>(config_, operable, std::move(transport));
    return session_->initialize();
}

damiao::Status MotorManager::scan_and_initialize(const ToolConfig& config,
    std::unique_ptr<damiao::ICanTransport> transport)
{
    config_ = config;
    const auto scanned = scan_motors(config_, std::move(transport));
    if (scanned.status.code != damiao::ErrorCode::Ok)
    {
        return scanned.status;
    }
    motors_ = scanned.motors;
    build_session_mapping(motors_, list_to_session_);
    selected_list_index_ = first_operable_index(motors_);
    return rebuild_session(std::make_unique<damiao::SocketCanTransport>());
}

damiao::Status MotorManager::rescan(const ToolConfig& config,
    std::unique_ptr<damiao::ICanTransport> transport)
{
    if (session_ && (all_enabled() || control_active()))
    {
        return invalid_manager("Disable all motors before rescanning.");
    }
    if (session_)
    {
        const auto closed = session_->shutdown();
        if (closed.code != damiao::ErrorCode::Ok)
        {
            return closed;
        }
    }
    return scan_and_initialize(config, std::move(transport));
}

const std::vector<DiscoveredMotor>& MotorManager::motors() const noexcept
{
    return motors_;
}

const ToolConfig& MotorManager::config() const noexcept
{
    return config_;
}

std::size_t MotorManager::selected_list_index() const noexcept
{
    return selected_list_index_;
}

std::size_t MotorManager::operable_count() const noexcept
{
    std::size_t count = 0;
    for (const auto& motor : motors_)
    {
        if (motor.operable)
        {
            ++count;
        }
    }
    return count;
}

std::size_t MotorManager::registered_count() const noexcept
{
    return session_ ? session_->motor_count() : 0;
}

bool MotorManager::all_enabled() const noexcept
{
    return session_ && session_->all_enabled();
}

bool MotorManager::control_active() const noexcept
{
    return session_ && session_->control_active();
}

damiao::ErrorCode MotorManager::background_error() const noexcept
{
    return session_ ? session_->background_error() : damiao::ErrorCode::Ok;
}

damiao::BusState MotorManager::bus_state() const
{
    return session_ ? session_->bus_state() : damiao::BusState::Closed;
}

std::optional<damiao::MotorIndex> MotorManager::session_index_for_list(std::size_t list_index) const
{
    if (list_index >= list_to_session_.size())
    {
        return std::nullopt;
    }
    return list_to_session_[list_index];
}

damiao::Status MotorManager::select_motor(std::size_t list_index_one_based)
{
    if (list_index_one_based == 0 || list_index_one_based > motors_.size())
    {
        return invalid_manager("Motor number out of range.");
    }
    const std::size_t list_index = list_index_one_based - 1;
    if (!motors_[list_index].operable)
    {
        return invalid_manager("Selected motor is not registered.");
    }
    selected_list_index_ = list_index;
    return {};
}

void MotorManager::sync_discovered_from_session()
{
    if (!session_)
    {
        return;
    }
    for (std::size_t list_index = 0; list_index < motors_.size(); ++list_index)
    {
        const auto session_index = session_index_for_list(list_index);
        if (!session_index)
        {
            continue;
        }
        const auto& info = session_->motor_info(*session_index);
        motors_[list_index].raw_status = info.raw_status;
        motors_[list_index].output_position_rad = info.output_position_rad;
    }
}

void MotorManager::refresh_motor_display()
{
    if (session_)
    {
        session_->refresh_motor_cache();
    }
    sync_discovered_from_session();
}

std::vector<damiao::Result<damiao::MotorState>> MotorManager::status_all()
{
    std::vector<damiao::Result<damiao::MotorState>> results;
    if (!session_)
    {
        return results;
    }
    for (std::size_t list_index = 0; list_index < motors_.size(); ++list_index)
    {
        const auto session_index = session_index_for_list(list_index);
        if (!session_index)
        {
            results.push_back({{damiao::ErrorCode::Unsupported, "Motor is not registered."}, std::nullopt});
            continue;
        }
        results.push_back(session_->status(*session_index));
    }
    sync_discovered_from_session();
    return results;
}

damiao::Status MotorManager::enable_all()
{
    if (!session_)
    {
        return invalid_manager("Session is not initialized.");
    }
    const auto result = session_->enable_all();
    if (result.code == damiao::ErrorCode::Ok)
    {
        sync_discovered_from_session();
    }
    return result;
}

damiao::Status MotorManager::disable_all()
{
    if (!session_)
    {
        return invalid_manager("Session is not initialized.");
    }
    const auto result = session_->disable_all();
    if (result.code == damiao::ErrorCode::Ok)
    {
        sync_discovered_from_session();
    }
    return result;
}

damiao::Status MotorManager::drive_selected(double absolute_position_rad, double speed_rad_s)
{
    if (!session_)
    {
        return invalid_manager("Session is not initialized.");
    }
    const auto session_index = session_index_for_list(selected_list_index_);
    if (!session_index)
    {
        return invalid_manager("Selected motor is not registered.");
    }
    if (!motors_[selected_list_index_].drivable)
    {
        return invalid_manager("Selected motor is not in position-velocity mode.");
    }
    return session_->drive(*session_index, absolute_position_rad, speed_rad_s);
}

damiao::Status MotorManager::clear_error_selected()
{
    if (!session_)
    {
        return invalid_manager("Session is not initialized.");
    }
    const auto session_index = session_index_for_list(selected_list_index_);
    if (!session_index)
    {
        return invalid_manager("Selected motor is not registered.");
    }
    return session_->clear_error(*session_index);
}

damiao::Result<damiao::ControlMode> MotorManager::read_control_mode_selected()
{
    if (!session_)
    {
        return {{damiao::ErrorCode::Disconnected, "Session is not initialized."}, std::nullopt};
    }
    const auto session_index = session_index_for_list(selected_list_index_);
    if (!session_index)
    {
        return {{damiao::ErrorCode::InvalidCommand, "Selected motor is not registered."}, std::nullopt};
    }
    return session_->read_control_mode(*session_index);
}

damiao::Status MotorManager::set_control_mode_selected(damiao::ControlMode mode)
{
    if (!session_)
    {
        return invalid_manager("Session is not initialized.");
    }
    const auto session_index = session_index_for_list(selected_list_index_);
    if (!session_index)
    {
        return invalid_manager("Selected motor is not registered.");
    }
    const auto result = session_->set_control_mode(*session_index, mode);
    if (result.code != damiao::ErrorCode::Ok)
    {
        return result;
    }
    sync_discovered_mode(motors_[selected_list_index_], mode);
    return {};
}

damiao::Status MotorManager::save_parameters_selected()
{
    if (!session_)
    {
        return invalid_manager("Session is not initialized.");
    }
    const auto session_index = session_index_for_list(selected_list_index_);
    if (!session_index)
    {
        return invalid_manager("Selected motor is not registered.");
    }
    return session_->save_parameters(*session_index);
}

damiao::Status MotorManager::shutdown()
{
    if (!session_)
    {
        return {};
    }
    const auto result = session_->shutdown();
    session_.reset();
    return result;
}

}  // namespace damiao_tools
