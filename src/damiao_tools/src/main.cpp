#include "config.hpp"
#include "motor_test_session.hpp"

#include <damiao_core/protocol.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace
{

volatile std::sig_atomic_t shutdown_requested = 0;

void handle_signal(int)
{
    shutdown_requested = 1;
}

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

std::vector<std::string> split(const std::string& line)
{
    std::istringstream input(line);
    std::vector<std::string> fields;
    for (std::string field; input >> field;)
    {
        fields.push_back(field);
    }
    return fields;
}

bool parse_number(const std::string& text, double& value)
{
    try
    {
        std::size_t consumed = 0;
        value = std::stod(text, &consumed);
        return consumed == text.size();
    }
    catch (const std::exception&)
    {
        return false;
    }
}

void print_help()
{
    std::cout
        << "命令:\n"
        << "  status\n"
        << "  enable\n"
        << "  drive <absolute_position_rad> <speed_rad_s>\n"
        << "  disable\n"
        << "  help\n"
        << "  quit\n";
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

void install_signal_handlers()
{
    struct sigaction action{};
    action.sa_handler = handle_signal;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc == 2 && std::string(argv[1]) == "--help")
    {
        std::cout << "用法: damiao_motor_tool --file <motor.yaml>\n";
        return 0;
    }
    if (argc != 3 || std::string(argv[1]) != "--file")
    {
        std::cerr << "用法: damiao_motor_tool --file <motor.yaml>\n";
        return 2;
    }

    const auto loaded = damiao_tools::load_config(argv[2]);
    if (loaded.status.code != damiao::ErrorCode::Ok)
    {
        print_status_error(loaded.status);
        return 2;
    }

    damiao_tools::MotorTestSession session(loaded.config);
    const auto initialized = session.initialize();
    if (initialized.code != damiao::ErrorCode::Ok)
    {
        print_status_error(initialized);
        return 1;
    }

    install_signal_handlers();
    std::cout
        << "已连接单电机。位置单位为 rad，速度单位为 rad/s。\n"
        << "警告：本工具不会在故障或退出时自动失能；离开前请显式执行 disable。\n";
    print_help();

    damiao::ErrorCode reported_background_error = damiao::ErrorCode::Ok;
    std::string line;
    while (!shutdown_requested)
    {
        const auto background_error = session.background_error();
        if (background_error != damiao::ErrorCode::Ok
            && background_error != reported_background_error)
        {
            std::cerr << "周期发送已停止 [" << error_name(background_error)
                << "]；电机未自动失能，请检查状态并显式执行 disable。\n";
            reported_background_error = background_error;
        }

        std::cout << "damiao[" << bus_state_name(session.bus_state()) << "]> " << std::flush;
        if (!std::getline(std::cin, line))
        {
            break;
        }
        const auto fields = split(line);
        if (fields.empty())
        {
            continue;
        }

        damiao::Status result;
        if (fields[0] == "help" && fields.size() == 1)
        {
            print_help();
        }
        else if (fields[0] == "status" && fields.size() == 1)
        {
            print_motor_state(session.status());
        }
        else if (fields[0] == "enable" && fields.size() == 1)
        {
            result = session.enable();
            if (result.code == damiao::ErrorCode::Ok)
            {
                reported_background_error = damiao::ErrorCode::Ok;
                std::cout << "电机已使能，后台保持目标已启动。\n";
            }
            else
            {
                print_status_error(result);
            }
        }
        else if (fields[0] == "drive" && fields.size() == 3)
        {
            double position = 0.0;
            double speed = 0.0;
            if (!parse_number(fields[1], position) || !parse_number(fields[2], speed))
            {
                std::cerr << "错误：drive 参数必须是有效数字。\n";
                continue;
            }
            result = session.drive(position, speed);
            if (result.code == damiao::ErrorCode::Ok)
            {
                std::cout << "目标已更新：位置=" << position << " rad，最大速度="
                    << speed << " rad/s。\n";
            }
            else
            {
                print_status_error(result);
            }
        }
        else if (fields[0] == "disable" && fields.size() == 1)
        {
            result = session.disable();
            if (result.code == damiao::ErrorCode::Ok)
            {
                std::cout << "电机已确认失能。\n";
            }
            else
            {
                print_status_error(result);
            }
        }
        else if (fields[0] == "quit" && fields.size() == 1)
        {
            break;
        }
        else
        {
            std::cerr << "未知命令或参数数量错误；输入 help 查看用法。\n";
        }
    }

    if (session.motor_enabled())
    {
        std::cerr << "警告：退出时电机仍可能处于使能状态；按约定未发送自动失能命令。\n";
    }
    const auto closed = session.shutdown();
    if (closed.code != damiao::ErrorCode::Ok)
    {
        print_status_error(closed);
        return 1;
    }
    return 0;
}
