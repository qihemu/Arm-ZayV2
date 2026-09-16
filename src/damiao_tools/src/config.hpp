#pragma once

#include <damiao_core/types.hpp>

#include <string>

namespace damiao_tools
{

// 配置只描述一台电机及其输出轴运动边界，单位为 rad 和 rad/s。
struct ToolConfig
{
    std::string can_interface;
    std::uint16_t esc_id = 0;
    std::uint16_t mst_id = 0;
    double min_output_position_rad = 0.0;
    double max_output_position_rad = 0.0;
    double max_output_speed_rad_s = 0.0;
};

struct ConfigResult
{
    damiao::Status status;
    ToolConfig config;
};

// 严格读取单电机 YAML；未知、缺失或类型错误字段都会导致失败。
ConfigResult load_config(const std::string& path);

}  // namespace damiao_tools
