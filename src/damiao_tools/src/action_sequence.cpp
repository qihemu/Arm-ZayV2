#include "action_sequence.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <chrono>
#include <fstream>
#include <sstream>
#include <thread>

namespace damiao_tools
{
namespace
{

ActionSequenceParseResult parse_failure(const std::string& message)
{
    return {{damiao::ErrorCode::InvalidConfiguration, message}, {}};
}

std::string trim(const std::string& text)
{
    const auto begin = std::find_if_not(text.begin(), text.end(),
        [](unsigned char ch) { return std::isspace(ch); });
    const auto end = std::find_if_not(text.rbegin(), text.rend(),
        [](unsigned char ch) { return std::isspace(ch); }).base();
    if (begin >= end)
    {
        return {};
    }
    return std::string(begin, end);
}

std::string strip_comment(const std::string& line)
{
    const auto hash = line.find('#');
    if (hash == std::string::npos)
    {
        return line;
    }
    return line.substr(0, hash);
}

bool equals_ignore_case(const std::string& left, const std::string& right)
{
    if (left.size() != right.size())
    {
        return false;
    }
    for (std::size_t index = 0; index < left.size(); ++index)
    {
        if (std::tolower(static_cast<unsigned char>(left[index]))
            != std::tolower(static_cast<unsigned char>(right[index])))
        {
            return false;
        }
    }
    return true;
}

bool parse_positive_index(const std::string& text, std::size_t& value)
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

bool parse_finite_number(const std::string& text, double& value)
{
    try
    {
        std::size_t consumed = 0;
        value = std::stod(text, &consumed);
        if (consumed != text.size() || !std::isfinite(value))
        {
            return false;
        }
        return true;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

bool parse_key_value_token(const std::string& token, const char* key, double& value)
{
    const std::string prefix = std::string(key) + "=";
    if (token.size() <= prefix.size() || token.compare(0, prefix.size(), prefix) != 0)
    {
        return false;
    }
    return parse_finite_number(token.substr(prefix.size()), value);
}

bool parse_move_line(const std::string& line, ActionStep& step)
{
    std::istringstream stream(line);
    std::string motor_token;
    std::string pos_token;
    std::string ve_token;
    if (!(stream >> motor_token >> pos_token >> ve_token) || stream >> motor_token)
    {
        return false;
    }
    if (motor_token.size() < 2
        || (motor_token[0] != 'M' && motor_token[0] != 'm'))
    {
        return false;
    }
    std::size_t motor_one_based = 0;
    if (!parse_positive_index(motor_token.substr(1), motor_one_based))
    {
        return false;
    }
    double position = 0.0;
    double speed = 0.0;
    if (!parse_key_value_token(pos_token, "pos", position)
        || !parse_key_value_token(ve_token, "ve", speed))
    {
        return false;
    }
    step.kind = ActionStepKind::Move;
    step.motor_one_based = motor_one_based;
    step.position_rad = position;
    step.speed_rad_s = speed;
    return true;
}

bool parse_delay_line(const std::string& line, ActionStep& step)
{
    std::istringstream stream(line);
    std::string keyword;
    std::string delay_text;
    if (!(stream >> keyword >> delay_text) || stream >> keyword)
    {
        return false;
    }
    if (!equals_ignore_case(keyword, "delay"))
    {
        return false;
    }
    std::size_t delay_ms = 0;
    if (!parse_positive_index(delay_text, delay_ms) || delay_ms > 3'600'000)
    {
        return false;
    }
    step.kind = ActionStepKind::Delay;
    step.delay_ms = static_cast<std::uint32_t>(delay_ms);
    return true;
}

void default_wait(std::uint32_t delay_ms, const std::atomic<bool>& cancel_requested)
{
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(delay_ms);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (cancel_requested.load())
        {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

}  // namespace

ActionSequenceParseResult parse_action_sequence_file(const std::string& path)
{
    std::ifstream input(path);
    if (!input)
    {
        return parse_failure("Cannot open action sequence file: " + path);
    }

    ActionSequenceParseResult result;
    std::string raw_line;
    std::size_t line_number = 0;
    while (std::getline(input, raw_line))
    {
        ++line_number;
        const std::string line = trim(strip_comment(raw_line));
        if (line.empty())
        {
            continue;
        }

        ActionStep step;
        if (parse_move_line(line, step))
        {
            result.steps.push_back(step);
            continue;
        }
        if (parse_delay_line(line, step))
        {
            result.steps.push_back(step);
            continue;
        }
        return parse_failure("Invalid action sequence syntax at line " + std::to_string(line_number)
            + ": " + line);
    }
    if (result.steps.empty())
    {
        return parse_failure("Action sequence file contains no steps: " + path);
    }
    result.status = {};
    return result;
}

std::string resolve_sequence_path(const std::string& config_directory, const std::string& path)
{
    if (path.empty())
    {
        return {};
    }
    if (!path.empty() && path[0] == '/')
    {
        return path;
    }
    if (config_directory.empty())
    {
        return path;
    }
    if (config_directory.back() == '/')
    {
        return config_directory + path;
    }
    return config_directory + "/" + path;
}

damiao::Status run_action_sequence(const std::vector<ActionStep>& steps,
    const ActionSequenceCallbacks& callbacks, const std::atomic<bool>& cancel_requested)
{
    if (!callbacks.move || !callbacks.wait)
    {
        return {damiao::ErrorCode::InvalidCommand, "Action sequence callbacks are not set."};
    }
    for (std::size_t index = 0; index < steps.size(); ++index)
    {
        if (cancel_requested.load())
        {
            return {damiao::ErrorCode::InvalidCommand, "Action sequence cancelled."};
        }
        const auto& step = steps[index];
        if (step.kind == ActionStepKind::Move)
        {
            const auto move_result = callbacks.move(step.motor_one_based, step.position_rad,
                step.speed_rad_s);
            if (move_result.code != damiao::ErrorCode::Ok)
            {
                return {move_result.code,
                    "Action sequence step " + std::to_string(index + 1) + " failed: "
                        + move_result.message};
            }
            continue;
        }
        callbacks.wait(step.delay_ms, cancel_requested);
    }
    if (cancel_requested.load())
    {
        return {damiao::ErrorCode::InvalidCommand, "Action sequence cancelled."};
    }
    return {};
}

damiao::Status run_action_sequence(const std::vector<ActionStep>& steps, MotorManager& manager,
    const std::atomic<bool>& cancel_requested)
{
    ActionSequenceCallbacks callbacks;
    callbacks.move = [&manager](std::size_t motor_one_based, double position_rad, double speed_rad_s)
    {
        return manager.drive_motor(motor_one_based, position_rad, speed_rad_s);
    };
    callbacks.wait = [](std::uint32_t delay_ms, const std::atomic<bool>& cancel)
    {
        default_wait(delay_ms, cancel);
    };
    return run_action_sequence(steps, callbacks, cancel_requested);
}

}  // namespace damiao_tools
