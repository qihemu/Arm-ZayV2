#include "console_output.hpp"

#include <damiao_core/protocol.hpp>

#include <chrono>
#include <iomanip>
#include <iostream>

namespace damiao_tools
{

const char* error_name(damiao::ErrorCode code)
{
    using damiao::ErrorCode;
    switch (code)
    {
        case ErrorCode::Ok: return "Ok";
        case ErrorCode::InvalidConfiguration: return "InvalidConfiguration";
        case ErrorCode::InvalidCommand: return "InvalidCommand";
        case ErrorCode::Timeout: return "Timeout";
        case ErrorCode::Disconnected: return "Disconnected";
        case ErrorCode::BusError: return "BusError";
        case ErrorCode::StaleFeedback: return "StaleFeedback";
        case ErrorCode::MotorFault: return "MotorFault";
        case ErrorCode::OwnershipConflict: return "OwnershipConflict";
        case ErrorCode::AmbiguousReply: return "AmbiguousReply";
        case ErrorCode::PartialFailure: return "PartialFailure";
        case ErrorCode::Unsupported: return "Unsupported";
        case ErrorCode::InvalidFrame: return "InvalidFrame";
        case ErrorCode::WouldBlock: return "WouldBlock";
        case ErrorCode::NotExecuted: return "NotExecuted";
    }
    return "Unknown";
}

const char* bus_state_name(damiao::BusState state)
{
    switch (state)
    {
        case damiao::BusState::Closed: return "CLOSED";
        case damiao::BusState::Maintenance: return "MAINTENANCE";
        case damiao::BusState::Control: return "CONTROL";
        case damiao::BusState::Fault: return "FAULT";
    }
    return "UNKNOWN";
}

void print_status_error(const damiao::Status& status)
{
    std::cerr << "错误 [" << error_name(status.code) << "]";
    if (!status.message.empty())
    {
        std::cerr << ": " << status.message;
    }
    std::cerr << '\n';
}

void print_motor_state(const damiao::Result<damiao::MotorState>& result)
{
    if (!result.value)
    {
        print_status_error(result.status);
        return;
    }
    const auto& state = *result.value;
    const auto now = damiao::SteadyClock::now();
    const auto age = now >= state.received_at
        ? std::chrono::duration_cast<std::chrono::milliseconds>(now - state.received_at).count()
        : 0;
    std::cout << std::fixed << std::setprecision(6)
        << "状态=" << static_cast<unsigned int>(state.raw_status)
        << " (" << damiao::DamiaoProtocol::status_description(state.raw_status) << ")"
        << ", 位置=" << state.output_position_rad << " rad"
        << ", 速度=" << state.output_velocity_rad_s << " rad/s"
        << ", 估计力矩=" << state.reported_torque_nm << " N*m"
        << ", MOS温度=" << static_cast<unsigned int>(state.mos_temperature_c) << " C"
        << ", 转子温度=" << static_cast<unsigned int>(state.rotor_temperature_c) << " C"
        << ", 反馈年龄=" << age << " ms\n";
    if (result.status.code != damiao::ErrorCode::Ok)
    {
        print_status_error(result.status);
    }
}

void print_motor_list(const std::vector<DiscoveredMotor>& motors, const std::string& can_interface,
    std::size_t registered_count, std::size_t selected_list_index)
{
    std::cout << "[" << can_interface << "] 已注册 " << registered_count << "/" << motors.size() << " 台\n";
    for (std::size_t index = 0; index < motors.size(); ++index)
    {
        const auto& motor = motors[index];
        std::cout << "M" << (index + 1)
            << (index == selected_list_index ? " *" : "")
            << "：esc_id=" << motor.esc_id
            << ", mst_id=" << motor.mst_id
            << ", mode=" << control_mode_name(motor.mode)
            << ", PMAX=" << motor.pmax_rad
            << ", VMAX=" << motor.vmax_rad_s
            << ", TMAX=" << motor.tmax_nm
            << ", fw=" << motor.firmware_version
            << ", pos=" << motor.output_position_rad << " rad"
            << ", status=" << (motor.raw_status == 1 ? "使能" : "失能");
        if (!motor.operable)
        {
            std::cout << " [不可操作";
            if (!motor.inoperable_reason.empty())
            {
                std::cout << ": " << motor.inoperable_reason;
            }
            std::cout << "]";
        }
        std::cout << '\n';
    }
}

}  // namespace damiao_tools
