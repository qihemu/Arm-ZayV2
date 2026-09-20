#pragma once

#include "motor_manager.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace damiao_tools
{

enum class ActionStepKind
{
    Move,
    Delay,
};

struct ActionStep
{
    ActionStepKind kind = ActionStepKind::Move;
    std::size_t motor_one_based = 0;
    double position_rad = 0.0;
    double speed_rad_s = 0.0;
    std::uint32_t delay_ms = 0;
};

struct ActionSequenceParseResult
{
    damiao::Status status;
    std::vector<ActionStep> steps;
};

struct ActionSequenceCallbacks
{
    std::function<damiao::Status(std::size_t motor_one_based, double position_rad, double speed_rad_s)> move;
    std::function<void(std::uint32_t delay_ms, const std::atomic<bool>& cancel_requested)> wait;
};

// 解析动作序列文本；错误信息含行号。
ActionSequenceParseResult parse_action_sequence_file(const std::string& path);

// 将配置中的相对路径解析为绝对路径（相对 motor.yaml 所在目录）。
std::string resolve_sequence_path(const std::string& config_directory, const std::string& path);

damiao::Status run_action_sequence(const std::vector<ActionStep>& steps,
    const ActionSequenceCallbacks& callbacks, const std::atomic<bool>& cancel_requested);

damiao::Status run_action_sequence(const std::vector<ActionStep>& steps, MotorManager& manager,
    const std::atomic<bool>& cancel_requested);

}  // namespace damiao_tools
