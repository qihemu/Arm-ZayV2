#include "motor_bus_session.hpp"

#include <cmath>
#include <exception>
#include <sstream>
#include <system_error>

namespace damiao_tools
{
namespace
{

using namespace std::chrono_literals;

constexpr auto control_period = 10ms;
constexpr auto feedback_timeout = 100ms;
constexpr auto management_quiet_period = 5ms;
constexpr auto operation_timeout = 2s;

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
        return invalid_session("No operable motors to initialize.");
    }

    for (std::size_t index = 0; index < motors_.size(); ++index)
    {
        const auto& discovered = motors_[index];
        damiao::MotorConfig motor;
        std::ostringstream name;
        name << "motor_" << discovered.esc_id;
        motor.name = name.str();
        motor.address = {discovered.esc_id, discovered.mst_id};
        motor.mode = damiao::ControlMode::PositionVelocity;
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
        if (!synchronized.value || synchronized.value->mode != damiao::ControlMode::PositionVelocity)
        {
            bus_.close();
            return {damiao::ErrorCode::Unsupported,
                "Motor must use position-velocity mode; this tool does not switch modes."};
        }
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
    return bus_.clear_error(index, operation_deadline());
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

}  // namespace damiao_tools
