#include "motor_bus_session.hpp"

#include <cmath>
#include <exception>
#include <sstream>
#include <system_error>
#include <thread>

namespace damiao_tools
{
namespace
{

using namespace std::chrono_literals;

constexpr auto control_period = 10ms;
constexpr auto feedback_timeout = 100ms;
constexpr auto management_quiet_period = 5ms;
constexpr auto operation_timeout = 2s;
constexpr auto flash_settle_period = 50ms;
constexpr auto flash_resync_timeout = 5s;
constexpr auto flash_resync_attempt = 500ms;

damiao::Status invalid_session(const char* message)
{
    return {damiao::ErrorCode::InvalidCommand, message};
}

}  // namespace

MotorBusSession::MotorBusSession(const ToolConfig& config, const std::vector<DiscoveredMotor>& motors,
    std::unique_ptr<damiao::ICanTransport> transport)
    : config_(config), motors_(motors), bus_(std::move(transport))
{
}

MotorBusSession::~MotorBusSession()
{
    // 析构只释放主机资源，不隐式失能电机。
    shutdown();
}

damiao::Deadline MotorBusSession::operation_deadline() const
{
    return damiao::SteadyClock::now() + operation_timeout;
}

damiao::Status MotorBusSession::initialize()
{
    if (initialized_)
    {
        return invalid_session("Session is already initialized.");
    }
    if (motors_.empty())
    {
        return invalid_session("No registered motors to initialize.");
    }

    for (std::size_t index = 0; index < motors_.size(); ++index)
    {
        const auto& discovered = motors_[index];
        damiao::MotorConfig motor;
        std::ostringstream name;
        name << "motor_" << discovered.esc_id;
        motor.name = name.str();
        motor.address = {discovered.esc_id, discovered.mst_id};
        motor.mode = discovered.mode;
        motor.min_output_position_rad = config_.min_output_position_rad;
        motor.max_output_position_rad = config_.max_output_position_rad;
        motor.max_output_speed_rad_s = config_.max_output_speed_rad_s;
        const auto registration = bus_.register_motor(motor);
        if (registration.status.code != damiao::ErrorCode::Ok)
        {
            return registration.status;
        }
    }

    damiao::BusConfig bus_config;
    bus_config.transport.interface_name = config_.can_interface;
    bus_config.feedback_timeout = feedback_timeout;
    bus_config.management_quiet_period = management_quiet_period;
    auto result = bus_.open(bus_config);
    if (result.code != damiao::ErrorCode::Ok)
    {
        return result;
    }

    commands_.resize(motors_.size());
    for (damiao::MotorIndex index = 0; index < motors_.size(); ++index)
    {
        result = bus_.synchronize_motor(index, operation_deadline());
        if (result.code != damiao::ErrorCode::Ok)
        {
            bus_.close();
            return result;
        }
        const auto synchronized = bus_.motor_config(index);
        if (!synchronized.value)
        {
            bus_.close();
            return {damiao::ErrorCode::InvalidConfiguration, "Motor configuration unavailable after synchronize."};
        }
        motors_[index].mode = synchronized.value->mode;
        motors_[index].drivable = synchronized.value->mode == damiao::ControlMode::PositionVelocity;
        const auto current = bus_.query_state(index, operation_deadline());
        if (!current.value)
        {
            bus_.close();
            return current.status;
        }
        motors_[index].raw_status = current.value->raw_status;
        motors_[index].output_position_rad = current.value->output_position_rad;
        commands_[index].output_position_rad = current.value->output_position_rad;
        commands_[index].max_output_speed_rad_s = config_.max_output_speed_rad_s;
    }

    initialized_ = true;
    all_enabled_ = false;
    return {};
}

damiao::Result<damiao::MotorState> MotorBusSession::status(damiao::MotorIndex index)
{
    if (!initialized_)
    {
        return {{damiao::ErrorCode::Disconnected, "Session is not initialized."}, std::nullopt};
    }
    if (index >= motors_.size())
    {
        return {{damiao::ErrorCode::InvalidCommand, "Unknown motor index."}, std::nullopt};
    }
    damiao::Result<damiao::MotorState> result;
    if (bus_.state() == damiao::BusState::Control)
    {
        result = bus_.snapshot(index);
    }
    else
    {
        result = bus_.query_state(index, operation_deadline());
    }
    if (result.value && result.status.code != damiao::ErrorCode::StaleFeedback)
    {
        motors_[index].raw_status = result.value->raw_status;
        motors_[index].output_position_rad = result.value->output_position_rad;
    }
    return result;
}

damiao::Status MotorBusSession::enable_all()
{
    if (!initialized_)
    {
        return invalid_session("Initialize the session before enabling motors.");
    }
    if (control_active_ || control_thread_.joinable())
    {
        return invalid_session("Control is already active.");
    }
    if (bus_.state() == damiao::BusState::Fault)
    {
        const auto recovered = bus_.recover_maintenance();
        if (recovered.code != damiao::ErrorCode::Ok)
        {
            return recovered;
        }
    }
    for (damiao::MotorIndex index = 0; index < motors_.size(); ++index)
    {
        const auto config = bus_.motor_config(index);
        if (!config.value || config.value->mode != damiao::ControlMode::PositionVelocity)
        {
            return invalid_session("全部注册电机须为位置速度模式方可使能驱动。");
        }
    }

    for (damiao::MotorIndex index = 0; index < motors_.size(); ++index)
    {
        const auto current = bus_.query_state(index, operation_deadline());
        if (current.status.code != damiao::ErrorCode::Ok || !current.value)
        {
            return current.status;
        }
        if (current.value->raw_status != 0)
        {
            return invalid_session("Enable requires all motors to be disabled.");
        }
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            commands_[index].output_position_rad = current.value->output_position_rad;
            commands_[index].max_output_speed_rad_s = config_.max_output_speed_rad_s;
        }
    }

    for (damiao::MotorIndex index = 0; index < motors_.size(); ++index)
    {
        const auto result = bus_.enable(index, operation_deadline());
        if (result.code != damiao::ErrorCode::Ok)
        {
            return result;
        }
    }

    auto result = bus_.begin_control();
    if (result.code != damiao::ErrorCode::Ok)
    {
        return result;
    }

    background_error_ = damiao::ErrorCode::Ok;
    stop_requested_ = false;
    all_enabled_ = true;
    control_active_ = true;
    try
    {
        control_thread_ = std::thread(&MotorBusSession::control_loop, this);
    }
    catch (const std::system_error& error)
    {
        stop_requested_ = true;
        control_active_ = false;
        all_enabled_ = false;
        bus_.end_control();
        return {damiao::ErrorCode::Disconnected, error.what()};
    }
    refresh_motor_cache();
    return {};
}

damiao::Status MotorBusSession::drive(damiao::MotorIndex index, double absolute_position_rad,
    double speed_rad_s)
{
    if (!control_active_ || !control_thread_.joinable())
    {
        return invalid_session("请先使能全部电机");
    }
    if (index >= motors_.size())
    {
        return invalid_session("Unknown motor index.");
    }
    if (!motors_[index].drivable)
    {
        return invalid_session("Only position-velocity motors can be driven.");
    }
    if (!std::isfinite(absolute_position_rad) || !std::isfinite(speed_rad_s)
        || absolute_position_rad < config_.min_output_position_rad
        || absolute_position_rad > config_.max_output_position_rad
        || speed_rad_s <= 0.0 || speed_rad_s > config_.max_output_speed_rad_s)
    {
        return invalid_session("Drive target is non-finite or outside the configured position/speed limits.");
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    commands_[index] = {absolute_position_rad, speed_rad_s};
    return {};
}

damiao::Status MotorBusSession::clear_error(damiao::MotorIndex index)
{
    if (!initialized_)
    {
        return invalid_session("Session is not initialized.");
    }
    if (index >= motors_.size())
    {
        return invalid_session("Unknown motor index.");
    }
    if (control_active_ || all_enabled_)
    {
        return invalid_session("Disable all motors before clearing errors.");
    }
    const auto gate = ensure_all_motors_disabled();
    if (gate.code != damiao::ErrorCode::Ok)
    {
        return gate;
    }
    return bus_.clear_error(index, operation_deadline());
}

damiao::Result<damiao::ControlMode> MotorBusSession::read_control_mode(damiao::MotorIndex index)
{
    if (!initialized_)
    {
        return {{damiao::ErrorCode::Disconnected, "Session is not initialized."}, std::nullopt};
    }
    if (index >= motors_.size())
    {
        return {{damiao::ErrorCode::InvalidCommand, "Unknown motor index."}, std::nullopt};
    }
    if (control_active_ || all_enabled_)
    {
        return {{damiao::ErrorCode::InvalidCommand,
            "Disable all motors before reading control mode."}, std::nullopt};
    }
    return bus_.read_control_mode(index, operation_deadline());
}

damiao::Status MotorBusSession::set_control_mode(damiao::MotorIndex index, damiao::ControlMode mode)
{
    if (!initialized_)
    {
        return invalid_session("Session is not initialized.");
    }
    if (index >= motors_.size())
    {
        return invalid_session("Unknown motor index.");
    }
    if (control_active_ || all_enabled_)
    {
        return invalid_session("Disable all motors before changing control mode.");
    }
    const auto gate = ensure_all_motors_disabled();
    if (gate.code != damiao::ErrorCode::Ok)
    {
        return gate;
    }
    const auto current_mode = bus_.read_control_mode(index, operation_deadline());
    if (current_mode.status.code != damiao::ErrorCode::Ok || !current_mode.value)
    {
        return current_mode.status;
    }
    if (*current_mode.value == mode)
    {
        motors_[index].mode = mode;
        return {};
    }
    const auto result = bus_.set_control_mode(index, mode, operation_deadline());
    if (result.code != damiao::ErrorCode::Ok)
    {
        return result;
    }
    const auto readback = bus_.read_control_mode(index, operation_deadline());
    if (readback.status.code != damiao::ErrorCode::Ok || !readback.value)
    {
        return readback.status;
    }
    if (*readback.value != mode)
    {
        return {damiao::ErrorCode::AmbiguousReply, "Control mode readback differs from requested value."};
    }
    motors_[index].mode = mode;
    motors_[index].drivable = mode == damiao::ControlMode::PositionVelocity;
    return {};
}

damiao::Status MotorBusSession::save_parameters(damiao::MotorIndex index)
{
    if (!initialized_)
    {
        return invalid_session("Session is not initialized.");
    }
    if (index >= motors_.size())
    {
        return invalid_session("Unknown motor index.");
    }
    if (control_active_ || all_enabled_)
    {
        return invalid_session("Disable all motors before saving parameters.");
    }
    const auto gate = ensure_all_motors_disabled();
    if (gate.code != damiao::ErrorCode::Ok)
    {
        return gate;
    }
    const auto result = bus_.save_parameters(index, operation_deadline());
    if (result.code != damiao::ErrorCode::Ok)
    {
        return result;
    }
    return resync_motor_after_save(index);
}

void MotorBusSession::control_loop()
{
    auto next_cycle = damiao::SteadyClock::now();
    while (!stop_requested_)
    {
        std::vector<damiao::PositionVelocityCommand> commands;
        commands.resize(motors_.size());
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            commands = commands_;
        }
        std::vector<damiao::ErrorCode> per_motor(motors_.size(), damiao::ErrorCode::NotExecuted);
        const auto result = bus_.send_position_velocity_batch(
            commands.data(), commands.size(), per_motor.data());
        const bool transient_busy = result == damiao::ErrorCode::WouldBlock
            && bus_.state() == damiao::BusState::Control;
        if (result != damiao::ErrorCode::Ok && !transient_busy)
        {
            background_error_ = result;
            stop_requested_ = true;
            break;
        }

        next_cycle += control_period;
        const auto now = damiao::SteadyClock::now();
        if (next_cycle <= now)
        {
            next_cycle = now + control_period;
        }
        std::this_thread::sleep_until(next_cycle);
    }

    if (control_active_.exchange(false))
    {
        bus_.end_control();
    }
}

damiao::Status MotorBusSession::stop_control_loop()
{
    stop_requested_ = true;
    if (control_thread_.joinable())
    {
        control_thread_.join();
    }
    if (control_active_.exchange(false))
    {
        return bus_.end_control();
    }
    return {};
}

damiao::Status MotorBusSession::disable_all()
{
    if (!initialized_)
    {
        return invalid_session("Session is not initialized.");
    }
    const auto stop_status = stop_control_loop();
    if (stop_status.code != damiao::ErrorCode::Ok && bus_.state() != damiao::BusState::Fault)
    {
        return stop_status;
    }
    for (damiao::MotorIndex index = 0; index < motors_.size(); ++index)
    {
        const auto result = bus_.disable(index, operation_deadline());
        if (result.code != damiao::ErrorCode::Ok)
        {
            return result;
        }
    }
    all_enabled_ = false;
    for (damiao::MotorIndex index = 0; index < motors_.size(); ++index)
    {
        const auto current = bus_.query_state(index, operation_deadline());
        if (current.value && current.status.code == damiao::ErrorCode::Ok)
        {
            motors_[index].raw_status = current.value->raw_status;
            motors_[index].output_position_rad = current.value->output_position_rad;
        }
        else
        {
            motors_[index].raw_status = 0;
        }
    }
    return {};
}

damiao::Status MotorBusSession::shutdown()
{
    stop_control_loop();
    if (!initialized_ && bus_.state() == damiao::BusState::Closed)
    {
        return {};
    }
    const auto result = bus_.close();
    initialized_ = false;
    all_enabled_ = false;
    return result;
}

std::size_t MotorBusSession::motor_count() const noexcept
{
    return motors_.size();
}

bool MotorBusSession::all_enabled() const noexcept
{
    return all_enabled_;
}

bool MotorBusSession::control_active() const noexcept
{
    return control_active_;
}

damiao::ErrorCode MotorBusSession::background_error() const noexcept
{
    return background_error_;
}

damiao::BusState MotorBusSession::bus_state() const
{
    return bus_.state();
}

const DiscoveredMotor& MotorBusSession::motor_info(damiao::MotorIndex index) const
{
    return motors_.at(index);
}

damiao::Status MotorBusSession::ensure_all_motors_disabled()
{
    for (damiao::MotorIndex index = 0; index < motors_.size(); ++index)
    {
        const auto current = bus_.query_state(index, operation_deadline());
        if (current.status.code != damiao::ErrorCode::Ok || !current.value)
        {
            if (bus_.state() == damiao::BusState::Fault)
            {
                const auto recovered = bus_.recover_maintenance();
                if (recovered.code != damiao::ErrorCode::Ok)
                {
                    return recovered;
                }
                const auto retry = bus_.query_state(index, operation_deadline());
                if (retry.status.code != damiao::ErrorCode::Ok || !retry.value)
                {
                    return retry.status;
                }
                if (retry.value->raw_status != 0)
                {
                    return invalid_session("Maintenance operation requires all motors to be disabled.");
                }
                motors_[index].raw_status = retry.value->raw_status;
                motors_[index].output_position_rad = retry.value->output_position_rad;
                continue;
            }
            return current.status;
        }
        if (current.value->raw_status != 0)
        {
            return invalid_session("Maintenance operation requires all motors to be disabled.");
        }
        motors_[index].raw_status = current.value->raw_status;
        motors_[index].output_position_rad = current.value->output_position_rad;
    }
    return {};
}

damiao::Status MotorBusSession::resync_motor_after_save(damiao::MotorIndex index)
{
    std::this_thread::sleep_for(flash_settle_period);
    const auto deadline = damiao::SteadyClock::now() + flash_resync_timeout;
    while (damiao::SteadyClock::now() < deadline)
    {
        if (bus_.state() == damiao::BusState::Fault)
        {
            const auto recovered = bus_.recover_maintenance();
            if (recovered.code != damiao::ErrorCode::Ok)
            {
                return recovered;
            }
        }
        const auto remaining = deadline - damiao::SteadyClock::now();
        if (remaining <= std::chrono::milliseconds(0))
        {
            break;
        }
        auto attempt_deadline = damiao::SteadyClock::now() + flash_resync_attempt;
        if (attempt_deadline > deadline)
        {
            attempt_deadline = deadline;
        }
        const auto sync = bus_.synchronize_motor(index, attempt_deadline);
        if (sync.code != damiao::ErrorCode::Ok)
        {
            std::this_thread::sleep_for(flash_settle_period);
            continue;
        }
        const auto current = bus_.query_state(index, attempt_deadline);
        if (current.status.code != damiao::ErrorCode::Ok || !current.value)
        {
            std::this_thread::sleep_for(flash_settle_period);
            continue;
        }
        motors_[index].raw_status = current.value->raw_status;
        motors_[index].output_position_rad = current.value->output_position_rad;
        const auto config = bus_.motor_config(index);
        if (config.value)
        {
            motors_[index].mode = config.value->mode;
            motors_[index].drivable = config.value->mode == damiao::ControlMode::PositionVelocity;
        }
        return {};
    }
    return {damiao::ErrorCode::Timeout, "Motor did not recover after parameter save."};
}

void MotorBusSession::refresh_motor_cache()
{
    if (!initialized_)
    {
        return;
    }
    if (bus_.state() == damiao::BusState::Control)
    {
        for (damiao::MotorIndex index = 0; index < motors_.size(); ++index)
        {
            const auto snapshot = bus_.snapshot(index);
            if (!snapshot.value)
            {
                continue;
            }
            motors_[index].raw_status = snapshot.value->raw_status;
            if (snapshot.status.code == damiao::ErrorCode::Ok)
            {
                motors_[index].output_position_rad = snapshot.value->output_position_rad;
            }
            else if (all_enabled_)
            {
                motors_[index].raw_status = 1;
            }
        }
        return;
    }
    if (bus_.state() == damiao::BusState::Fault)
    {
        bus_.recover_maintenance();
    }
    for (damiao::MotorIndex index = 0; index < motors_.size(); ++index)
    {
        const auto current = bus_.query_state(index, operation_deadline());
        if (current.value && current.status.code == damiao::ErrorCode::Ok)
        {
            motors_[index].raw_status = current.value->raw_status;
            motors_[index].output_position_rad = current.value->output_position_rad;
        }
    }
}

}  // namespace damiao_tools
