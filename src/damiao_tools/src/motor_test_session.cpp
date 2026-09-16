#include "motor_test_session.hpp"

#include <cmath>
#include <exception>
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

damiao::Status invalid(const char* message)
{
    return {damiao::ErrorCode::InvalidCommand, message};
}

}  // namespace

MotorTestSession::MotorTestSession(const ToolConfig& config,
    std::unique_ptr<damiao::ICanTransport> transport)
    : config_(config), bus_(std::move(transport))
{
}

MotorTestSession::~MotorTestSession()
{
    // 析构遵循工具约定：只释放主机资源，不隐式失能电机。
    shutdown();
}

damiao::Deadline MotorTestSession::operation_deadline() const
{
    return damiao::SteadyClock::now() + operation_timeout;
}

damiao::Status MotorTestSession::initialize()
{
    if (initialized_)
    {
        return invalid("Session is already initialized.");
    }

    damiao::MotorConfig motor;
    motor.name = "test_motor";
    motor.address = {config_.esc_id, config_.mst_id};
    motor.mode = damiao::ControlMode::PositionVelocity;
    motor.min_output_position_rad = config_.min_output_position_rad;
    motor.max_output_position_rad = config_.max_output_position_rad;
    motor.max_output_speed_rad_s = config_.max_output_speed_rad_s;
    const auto registration = bus_.register_motor(motor);
    if (registration.status.code != damiao::ErrorCode::Ok || !registration.value)
    {
        return registration.status;
    }
    motor_index_ = *registration.value;

    damiao::BusConfig bus_config;
    bus_config.transport.interface_name = config_.can_interface;
    bus_config.feedback_timeout = feedback_timeout;
    bus_config.management_quiet_period = management_quiet_period;
    auto result = bus_.open(bus_config);
    if (result.code != damiao::ErrorCode::Ok)
    {
        return result;
    }
    result = bus_.synchronize_motor(motor_index_, operation_deadline());
    if (result.code != damiao::ErrorCode::Ok)
    {
        bus_.close();
        return result;
    }
    const auto synchronized = bus_.motor_config(motor_index_);
    if (!synchronized.value || synchronized.value->mode != damiao::ControlMode::PositionVelocity)
    {
        bus_.close();
        return {damiao::ErrorCode::Unsupported,
            "Motor must already use position-velocity mode; this tool does not switch modes."};
    }
    const auto current = bus_.query_state(motor_index_, operation_deadline());
    if (!current.value)
    {
        bus_.close();
        return current.status;
    }
    motor_enabled_ = current.value->raw_status == 1;
    initialized_ = true;
    return current.status;
}

damiao::Result<damiao::MotorState> MotorTestSession::status()
{
    if (!initialized_)
    {
        return {{damiao::ErrorCode::Disconnected, "Session is not initialized."}, std::nullopt};
    }
    damiao::Result<damiao::MotorState> result;
    if (bus_.state() == damiao::BusState::Control)
    {
        result = bus_.snapshot(motor_index_);
    }
    else
    {
        result = bus_.query_state(motor_index_, operation_deadline());
    }
    if (result.value && result.status.code != damiao::ErrorCode::StaleFeedback)
    {
        motor_enabled_ = result.value->raw_status == 1;
    }
    return result;
}

damiao::Status MotorTestSession::enable()
{
    if (!initialized_)
    {
        return invalid("Initialize the session before enabling the motor.");
    }
    if (control_active_ || control_thread_.joinable())
    {
        return invalid("Control is already active.");
    }

    const auto current = bus_.query_state(motor_index_, operation_deadline());
    if (current.status.code != damiao::ErrorCode::Ok || !current.value)
    {
        return current.status;
    }
    if (current.value->raw_status != 0)
    {
        motor_enabled_ = current.value->raw_status == 1;
        return invalid("Enable requires a fresh disabled motor state.");
    }
    auto result = bus_.enable(motor_index_, operation_deadline());
    if (result.code != damiao::ErrorCode::Ok)
    {
        return result;
    }
    motor_enabled_ = true;
    result = bus_.begin_control();
    if (result.code != damiao::ErrorCode::Ok)
    {
        return result;
    }

    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        // 首帧保持使能前读取到的位置，避免工具主动把目标初始化为零。
        command_.output_position_rad = current.value->output_position_rad;
        command_.max_output_speed_rad_s = config_.max_output_speed_rad_s;
    }
    background_error_ = damiao::ErrorCode::Ok;
    stop_requested_ = false;
    control_active_ = true;
    try
    {
        control_thread_ = std::thread(&MotorTestSession::control_loop, this);
    }
    catch (const std::system_error& error)
    {
        stop_requested_ = true;
        control_active_ = false;
        bus_.end_control();
        return {damiao::ErrorCode::Disconnected, error.what()};
    }
    return {};
}

damiao::Status MotorTestSession::drive(double absolute_position_rad, double speed_rad_s)
{
    if (!control_active_ || !control_thread_.joinable())
    {
        return invalid("Enable the motor before setting a drive target.");
    }
    if (!std::isfinite(absolute_position_rad) || !std::isfinite(speed_rad_s)
        || absolute_position_rad < config_.min_output_position_rad
        || absolute_position_rad > config_.max_output_position_rad
        || speed_rad_s <= 0.0 || speed_rad_s > config_.max_output_speed_rad_s)
    {
        return invalid("Drive target is non-finite or outside the configured position/speed limits.");
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = {absolute_position_rad, speed_rad_s};
    return {};
}

void MotorTestSession::control_loop()
{
    auto next_cycle = damiao::SteadyClock::now();
    while (!stop_requested_)
    {
        damiao::PositionVelocityCommand command;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            command = command_;
        }
        damiao::ErrorCode per_motor = damiao::ErrorCode::NotExecuted;
        const auto result = bus_.send_position_velocity_batch(&command, 1, &per_motor);
        // 锁竞争产生的 WouldBlock 可跳过一周期；传输层 WouldBlock 会使总线锁存 Fault。
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

    // 发送故障只撤销主机控制许可；按照工具约定不自动失能电机。
    if (control_active_.exchange(false))
    {
        bus_.end_control();
    }
}

damiao::Status MotorTestSession::stop_control_loop()
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

damiao::Status MotorTestSession::disable()
{
    if (!initialized_)
    {
        return invalid("Session is not initialized.");
    }
    const auto stop_status = stop_control_loop();
    if (stop_status.code != damiao::ErrorCode::Ok && bus_.state() != damiao::BusState::Fault)
    {
        return stop_status;
    }
    const auto result = bus_.disable(motor_index_, operation_deadline());
    if (result.code == damiao::ErrorCode::Ok)
    {
        motor_enabled_ = false;
    }
    return result;
}

damiao::Status MotorTestSession::shutdown()
{
    stop_control_loop();
    if (!initialized_ && bus_.state() == damiao::BusState::Closed)
    {
        return {};
    }
    const auto result = bus_.close();
    initialized_ = false;
    // 关闭主机资源不代表电机已经失能，保留 motor_enabled_ 提示调用者。
    return result;
}

bool MotorTestSession::motor_enabled() const noexcept
{
    return motor_enabled_;
}

bool MotorTestSession::control_active() const noexcept
{
    return control_active_;
}

damiao::ErrorCode MotorTestSession::background_error() const noexcept
{
    return background_error_;
}

damiao::BusState MotorTestSession::bus_state() const
{
    return bus_.state();
}

}  // namespace damiao_tools
