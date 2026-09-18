#include "config.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <exception>
#include <string>

namespace damiao_tools
{
namespace
{

constexpr std::array<const char*, 4> required_keys{
    "can_interface",
    "min_output_position_rad",
    "max_output_position_rad",
    "max_output_speed_rad_s"
};

constexpr std::array<const char*, 3> optional_keys{
    "scan_esc_min",
    "scan_esc_max",
    "scan_timeout_ms"
};

bool known_key(const std::string& key)
{
    for (const auto* allowed : required_keys)
    {
        if (key == allowed)
        {
            return true;
        }
    }
    for (const auto* allowed : optional_keys)
    {
        if (key == allowed)
        {
            return true;
        }
    }
    return false;
}

ConfigResult failure(const std::string& message)
{
    return {{damiao::ErrorCode::InvalidConfiguration, message}, {}};
}

}  // namespace

ConfigResult load_config(const std::string& path)
{
    try
    {
        const YAML::Node root = YAML::LoadFile(path);
        if (!root.IsMap())
        {
            return failure("Configuration root must be a map.");
        }
        for (const auto& entry : root)
        {
            if (!entry.first.IsScalar() || !known_key(entry.first.as<std::string>()))
            {
                return failure("Configuration contains an unknown field.");
            }
        }
        for (const auto* key : required_keys)
        {
            if (!root[key] || root[key].IsNull())
            {
                return failure(std::string("Missing required field: ") + key);
            }
        }

        ToolConfig config;
        config.can_interface = root["can_interface"].as<std::string>();
        config.min_output_position_rad = root["min_output_position_rad"].as<double>();
        config.max_output_position_rad = root["max_output_position_rad"].as<double>();
        config.max_output_speed_rad_s = root["max_output_speed_rad_s"].as<double>();

        if (root["scan_esc_min"])
        {
            config.scan_esc_min = root["scan_esc_min"].as<std::uint16_t>();
        }
        if (root["scan_esc_max"])
        {
            config.scan_esc_max = root["scan_esc_max"].as<std::uint16_t>();
        }
        if (root["scan_timeout_ms"])
        {
            config.scan_timeout_ms = root["scan_timeout_ms"].as<std::uint32_t>();
        }

        if (config.can_interface.empty())
        {
            return failure("can_interface must not be empty.");
        }
        if (config.scan_esc_min < 1 || config.scan_esc_max > 15 || config.scan_esc_min > config.scan_esc_max)
        {
            return failure("scan_esc_min/max must be within 1..15 and ordered.");
        }
        if (config.scan_timeout_ms <= 0)
        {
            return failure("scan_timeout_ms must be positive.");
        }
        if (!std::isfinite(config.min_output_position_rad)
            || !std::isfinite(config.max_output_position_rad)
            || !std::isfinite(config.max_output_speed_rad_s)
            || config.min_output_position_rad >= config.max_output_position_rad
            || config.max_output_speed_rad_s <= 0.0)
        {
            return failure("Position limits must be finite and ordered; maximum speed must be finite and positive.");
        }
        return {{}, config};
    }
    catch (const YAML::Exception& error)
    {
        return failure(std::string("YAML error: ") + error.what());
    }
    catch (const std::exception& error)
    {
        return failure(std::string("Configuration error: ") + error.what());
    }
}

}  // namespace damiao_tools
