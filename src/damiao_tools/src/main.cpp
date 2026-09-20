#include "action_sequence.hpp"
#include "config.hpp"
#include "console_output.hpp"
#include "motor_manager.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>

namespace
{

volatile std::sig_atomic_t shutdown_requested = 0;
std::atomic<bool> sequence_cancel_requested{false};

void handle_signal(int)
{
    shutdown_requested = 1;
    sequence_cancel_requested.store(true);
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

bool parse_control_mode(const std::string& text, damiao::ControlMode& mode)
{
    std::size_t code = 0;
    if (!parse_index(text, code) || code > 4)
    {
        return false;
    }
    mode = static_cast<damiao::ControlMode>(code);
    return true;
}

void print_context(damiao_tools::MotorManager& manager)
{
    manager.refresh_motor_display();
    damiao_tools::print_motor_list(manager.motors(), manager.config().can_interface,
        manager.registered_count(), manager.selected_list_index());
}

void print_menu(damiao_tools::MotorManager& manager)
{
    print_context(manager);
    const auto enabled = manager.all_enabled();
    const auto registered = manager.registered_count();
    std::cout
        << "\n[" << manager.config().can_interface << "] 选中 M"
        << (manager.selected_list_index() + 1)
        << " | 已使能 " << (enabled ? registered : 0) << "/" << registered
        << " | 总线 " << damiao_tools::bus_state_name(manager.bus_state()) << "\n"
        << "1. 选择电机\n"
        << "2. 查询全部电机状态\n"
        << "3. 使能全部电机\n"
        << "4. 失能全部电机\n"
        << "5. 驱动选中电机到目标角度\n"
        << "6. 清错（当前选中电机）\n"
        << "7. 重新扫描总线\n"
        << "8. 修改选中电机控制模式（须已失能）\n"
        << "9. 保存参数到 Flash（须已失能）\n"
        << "10. 执行动作序列\n"
        << "0. 退出\n";
}

bool validate_move_steps(const std::vector<damiao_tools::ActionStep>& steps,
    damiao_tools::MotorManager& manager, std::size_t& failed_step)
{
    for (std::size_t index = 0; index < steps.size(); ++index)
    {
        const auto& step = steps[index];
        if (step.kind != damiao_tools::ActionStepKind::Move)
        {
            continue;
        }
        if (step.motor_one_based == 0 || step.motor_one_based > manager.motors().size())
        {
            failed_step = index + 1;
            return false;
        }
        const auto& motor = manager.motors()[step.motor_one_based - 1];
        if (!motor.operable || !motor.drivable)
        {
            failed_step = index + 1;
            return false;
        }
    }
    return true;
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
    const std::filesystem::path config_path(argv[2]);
    const std::string config_directory = config_path.has_parent_path()
        ? config_path.parent_path().string()
        : std::string(".");

    damiao_tools::MotorManager manager;
    const auto initialized = manager.scan_and_initialize(loaded.config);
    if (initialized.code != damiao::ErrorCode::Ok)
    {
        damiao_tools::print_status_error(initialized);
        return 1;
    }

    if (manager.registered_count() == 0)
    {
        print_context(manager);
        std::cerr << "未找到可注册电机，请检查 CAN 接线与接口配置。\n";
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
            const auto states = manager.status_all();
            for (std::size_t index = 0; index < states.size(); ++index)
            {
                std::cout << "M" << (index + 1) << ": ";
                damiao_tools::print_motor_state(states[index]);
            }
            continue;
        }
        if (choice == 3)
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
        if (choice == 4)
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
        if (choice == 5)
        {
            if (!manager.all_enabled())
            {
                std::cerr << "请先执行菜单 3 使能全部电机。\n";
                continue;
            }
            if (!manager.motors()[manager.selected_list_index()].drivable)
            {
                std::cerr << "当前选中电机非位置速度模式，无法驱动。\n";
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
        if (choice == 6)
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
        if (choice == 7)
        {
            const auto result = manager.rescan(loaded.config);
            if (result.code == damiao::ErrorCode::Ok)
            {
                std::cout << "重新扫描完成。\n";
            }
            else
            {
                damiao_tools::print_status_error(result);
            }
            continue;
        }
        if (choice == 8)
        {
            if (manager.all_enabled() || manager.control_active())
            {
                std::cerr << "请先执行菜单 4 失能全部电机。\n";
                continue;
            }
            const auto current_mode = manager.read_control_mode_selected();
            if (!current_mode.value)
            {
                damiao_tools::print_status_error(current_mode.status);
                continue;
            }
            std::cout << "当前模式: "
                << damiao_tools::control_mode_name(*current_mode.value) << '\n'
                << "1=MIT 2=PositionVelocity 3=Velocity 4=PositionCurrentLimit\n"
                << "输入模式编号> " << std::flush;
            if (!std::getline(std::cin, line))
            {
                break;
            }
            damiao::ControlMode target_mode = damiao::ControlMode::PositionVelocity;
            if (!parse_control_mode(line, target_mode))
            {
                std::cerr << "请输入 1 到 4 之间的模式编号。\n";
                continue;
            }
            const auto result = manager.set_control_mode_selected(target_mode);
            if (result.code == damiao::ErrorCode::Ok)
            {
                std::cout << "M" << (manager.selected_list_index() + 1)
                    << " 控制模式已更新为 "
                    << damiao_tools::control_mode_name(target_mode) << "。\n";
                if (target_mode != damiao::ControlMode::PositionVelocity)
                {
                    std::cout << "非位置速度模式不可使能或驱动；未写入 Flash，可用菜单 9 保存。\n";
                }
            }
            else
            {
                damiao_tools::print_status_error(result);
            }
            continue;
        }
        if (choice == 9)
        {
            if (manager.all_enabled() || manager.control_active())
            {
                std::cerr << "请先执行菜单 4 失能全部电机。\n";
                continue;
            }
            const auto result = manager.save_parameters_selected();
            if (result.code == damiao::ErrorCode::Ok)
            {
                std::cout << "M" << (manager.selected_list_index() + 1)
                    << " 参数已保存到 Flash。\n";
            }
            else
            {
                damiao_tools::print_status_error(result);
            }
            continue;
        }
        if (choice == 10)
        {
            if (!manager.all_enabled())
            {
                std::cerr << "请先执行菜单 3 使能全部电机。\n";
                continue;
            }
            std::string sequence_path;
            if (!loaded.config.action_sequence_file.empty())
            {
                sequence_path = damiao_tools::resolve_sequence_path(config_directory,
                    loaded.config.action_sequence_file);
            }
            else
            {
                std::cout << "未在配置中设置 action_sequence_file。\n"
                    << "输入动作序列文件路径（相对配置目录或绝对路径）> " << std::flush;
                if (!std::getline(std::cin, line))
                {
                    break;
                }
                line = damiao_tools::resolve_sequence_path(config_directory, line);
                if (line.empty())
                {
                    std::cerr << "路径不能为空。\n";
                    continue;
                }
                sequence_path = line;
            }
            const auto parsed = damiao_tools::parse_action_sequence_file(sequence_path);
            if (parsed.status.code != damiao::ErrorCode::Ok)
            {
                damiao_tools::print_status_error(parsed.status);
                continue;
            }
            std::size_t failed_step = 0;
            if (!validate_move_steps(parsed.steps, manager, failed_step))
            {
                std::cerr << "动作序列第 " << failed_step
                    << " 步电机不可用或未处于位置速度模式。\n";
                continue;
            }
            sequence_cancel_requested.store(false);
            std::cout << "开始执行动作序列：" << sequence_path << "（共 "
                << parsed.steps.size() << " 步）\n";
            const auto run_result = damiao_tools::run_action_sequence(parsed.steps, manager,
                sequence_cancel_requested);
            if (run_result.code == damiao::ErrorCode::Ok)
            {
                std::cout << "动作序列执行完成。\n";
            }
            else if (sequence_cancel_requested.load() || shutdown_requested)
            {
                std::cerr << "动作序列已中断。\n";
            }
            else
            {
                damiao_tools::print_status_error(run_result);
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
