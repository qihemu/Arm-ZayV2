#include "config.hpp"

#include <damiao_core/protocol.hpp>

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <string>

namespace damiao_tools
{
namespace
{

constexpr std::array<const char*, 6> allowed_keys{
    "can_interface",
    "esc_id",
    "mst_id",
    "min_output_position_rad",
    "max_output_position_rad",
    "max_output_speed_rad_s"
};

bool known_key(const std::string& key)
{
    for (const auto* allowed : allowed_keys)
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
        for (const auto* key : allowed_keys)
        {
            if (!root[key] || root[key].IsNull())
            {
                return failure(std::string("Missing required field: ") + key);
            }
        }

        ToolConfig config;
        config.can_interface = root["can_interface"].as<std::string>();
        const auto esc = root["esc_id"].as<std::uint32_t>();
        const auto mst = root["mst_id"].as<std::uint32_t>();
        if (esc > std::numeric_limits<std::uint16_t>::max()
            || mst > std::numeric_limits<std::uint16_t>::max())
        {
            return failure("Motor address does not fit the supported ID type.");
        }
        config.esc_id = static_cast<std::uint16_t>(esc);
        config.mst_id = static_cast<std::uint16_t>(mst);
        config.min_output_position_rad = root["min_output_position_rad"].as<double>();
        config.max_output_position_rad = root["max_output_position_rad"].as<double>();
        config.max_output_speed_rad_s = root["max_output_speed_rad_s"].as<double>();

        const damiao::MotorAddress address{config.esc_id, config.mst_id};
        if (config.can_interface.empty())
        {
            return failure("can_interface must not be empty.");
        }
        const auto address_status = damiao::DamiaoProtocol::validate_address(address);
        if (address_status.code != damiao::ErrorCode::Ok)
        {
            return {address_status, {}};
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
