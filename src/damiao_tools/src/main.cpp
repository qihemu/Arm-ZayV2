#include "config.hpp"
#include "console_output.hpp"
#include "motor_manager.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <sstream>
#include <string>

namespace
{

volatile std::sig_atomic_t shutdown_requested = 0;

void handle_signal(int)
{
    shutdown_requested = 1;
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

bool parse_index(const std::string& text, std::size_t& value)
{
    try
    {
        std::size_t consumed = 0;
        const unsigned long parsed = std::stoul(text, &consumed);
        if (consumed != text.size() || parsed == 0)
        {
            return false;
        }
        value = parsed;
        return true;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

void print_menu(const damiao_tools::MotorManager& manager)
{
    const auto enabled = manager.all_enabled();
    const auto registered = manager.registered_count();
    std::cout
        << "\n[" << manager.config().can_interface << "] 选中 M"
        << (manager.selected_list_index() + 1)
        << " | 已使能 " << (enabled ? registered : 0) << "/" << registered
        << " | 总线 " << damiao_tools::bus_state_name(manager.bus_state()) << "\n"
        << "1. 选择电机\n"
        << "2. 查询当前选中电机状态\n"
        << "3. 查询全部电机状态\n"
        << "4. 使能全部电机\n"
        << "5. 失能全部电机\n"
        << "6. 驱动选中电机到目标角度\n"
        << "7. 清错（当前选中电机）\n"
        << "8. 重新扫描总线\n"
        << "0. 退出\n";
}

void print_context(const damiao_tools::MotorManager& manager)
{
    damiao_tools::print_motor_list(manager.motors(), manager.config().can_interface,
        manager.registered_count(), manager.selected_list_index());
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
        damiao_tools::print_status_error(loaded.status);
        return 2;
    }

    damiao_tools::MotorManager manager;
    const auto initialized = manager.scan_and_initialize(loaded.config);
    if (initialized.code != damiao::ErrorCode::Ok)
    {
        damiao_tools::print_status_error(initialized);
        return 1;
    }

    print_context(manager);
    if (manager.operable_count() == 0)
    {
        std::cerr << "未找到可操作电机，请检查 CAN 接线与接口配置。\n";
        manager.shutdown();
        return 1;
    }

    install_signal_handlers();
    std::cout
        << "\n警告：本工具不会在故障或退出时自动失能；离开前请显式执行失能全部电机。\n"
        << "驱动前必须先使能全部电机。\n";

    damiao::ErrorCode reported_background_error = damiao::ErrorCode::Ok;
    while (!shutdown_requested)
    {
        const auto background_error = manager.background_error();
        if (background_error != damiao::ErrorCode::Ok
            && background_error != reported_background_error)
        {
            std::cerr << "周期发送已停止 [" << damiao_tools::error_name(background_error)
                << "]；电机未自动失能，请检查状态并显式执行失能全部电机。\n";
            reported_background_error = background_error;
        }

        print_menu(manager);
        std::cout << "请选择操作> " << std::flush;
        std::string line;
        if (!std::getline(std::cin, line))
        {
            break;
        }
        std::size_t choice = 0;
        if (!parse_index(line, choice))
        {
            std::cerr << "请输入有效数字。\n";
            continue;
        }

        if (choice == 0)
        {
            break;
        }
        if (choice == 1)
        {
            std::cout << "输入电机编号 M> " << std::flush;
            if (!std::getline(std::cin, line))
            {
                break;
            }
            std::size_t motor_number = 0;
            if (!parse_index(line, motor_number))
            {
                std::cerr << "请输入有效电机编号。\n";
                continue;
            }
            const auto result = manager.select_motor(motor_number);
            if (result.code == damiao::ErrorCode::Ok)
            {
                std::cout << "已选中 M" << motor_number << "。\n";
            }
            else
            {
                damiao_tools::print_status_error(result);
            }
            continue;
        }
        if (choice == 2)
        {
            damiao_tools::print_motor_state(manager.status_selected());
            continue;
        }
        if (choice == 3)
        {
            const auto states = manager.status_all();
            for (std::size_t index = 0; index < states.size(); ++index)
            {
                std::cout << "M" << (index + 1) << ": ";
                damiao_tools::print_motor_state(states[index]);
            }
            continue;
        }
        if (choice == 4)
        {
            const auto result = manager.enable_all();
            if (result.code == damiao::ErrorCode::Ok)
            {
                reported_background_error = damiao::ErrorCode::Ok;
                std::cout << "全部电机已使能，后台保持目标已启动。\n";
            }
            else
            {
                damiao_tools::print_status_error(result);
            }
            continue;
        }
        if (choice == 5)
        {
            const auto result = manager.disable_all();
            if (result.code == damiao::ErrorCode::Ok)
            {
                std::cout << "全部电机已确认失能。\n";
            }
            else
            {
                damiao_tools::print_status_error(result);
            }
            continue;
        }
        if (choice == 6)
        {
            if (!manager.all_enabled())
            {
                std::cerr << "请先执行菜单 4 使能全部电机。\n";
                continue;
            }
            std::cout << "输入目标位置(rad)> " << std::flush;
            if (!std::getline(std::cin, line))
            {
                break;
            }
            double position = 0.0;
            if (!parse_number(line, position))
            {
                std::cerr << "位置必须是有效数字。\n";
                continue;
            }
            std::cout << "输入最大速度(rad/s)> " << std::flush;
            if (!std::getline(std::cin, line))
            {
                break;
            }
            double speed = 0.0;
            if (!parse_number(line, speed))
            {
                std::cerr << "速度必须是有效数字。\n";
                continue;
            }
            const auto result = manager.drive_selected(position, speed);
            if (result.code == damiao::ErrorCode::Ok)
            {
                std::cout << "M" << (manager.selected_list_index() + 1)
                    << " 目标已更新：位置=" << position << " rad，最大速度="
                    << speed << " rad/s。\n";
            }
            else
            {
                damiao_tools::print_status_error(result);
            }
            continue;
        }
        if (choice == 7)
        {
            const auto result = manager.clear_error_selected();
            if (result.code == damiao::ErrorCode::Ok)
            {
                std::cout << "清错命令已确认。\n";
            }
            else
            {
                damiao_tools::print_status_error(result);
            }
            continue;
        }
        if (choice == 8)
        {
            const auto result = manager.rescan(loaded.config);
            if (result.code == damiao::ErrorCode::Ok)
            {
                print_context(manager);
                std::cout << "重新扫描完成。\n";
            }
            else
            {
                damiao_tools::print_status_error(result);
            }
            continue;
        }

        std::cerr << "未知菜单项。\n";
    }

    if (manager.all_enabled())
    {
        std::cerr << "警告：退出时电机仍可能处于使能状态；按约定未发送自动失能命令。\n";
    }
    const auto closed = manager.shutdown();
    if (closed.code != damiao::ErrorCode::Ok)
    {
        damiao_tools::print_status_error(closed);
        return 1;
    }
    return 0;
}
