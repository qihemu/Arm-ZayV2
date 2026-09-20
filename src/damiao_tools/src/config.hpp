#pragma once

#include <damiao_core/types.hpp>

#include <cstdint>
#include <string>

namespace damiao_tools
{

// 工具配置：单 CAN 接口、默认运动限位与扫描参数。
struct ToolConfig
{
    std::string can_interface;
    double min_output_position_rad = 0.0;
    double max_output_position_rad = 0.0;
    double max_output_speed_rad_s = 0.0;
    std::uint16_t scan_esc_min = 1;
    std::uint16_t scan_esc_max = 15;
    std::uint32_t scan_timeout_ms = 200;
    // 可选：相对 motor.yaml 目录或绝对路径的动作序列文本文件。
    std::string action_sequence_file;
};

struct ConfigResult
{
    damiao::Status status;
    ToolConfig config;
};

// 严格读取 YAML；未知、缺失或类型错误字段都会导致失败。
ConfigResult load_config(const std::string& path);

}  // namespace damiao_tools
