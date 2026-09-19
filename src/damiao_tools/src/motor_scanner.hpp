#pragma once

#include "config.hpp"

#include <damiao_core/bus.hpp>
#include <damiao_core/transport.hpp>
#include <damiao_core/types.hpp>

#include <memory>
#include <string>
#include <vector>

namespace damiao_tools
{

// 扫描阶段发现的电机信息；operable=已注册，drivable=可位置速度驱动。
struct DiscoveredMotor
{
    std::uint16_t esc_id = 0;
    std::uint16_t mst_id = 0;
    damiao::ControlMode mode = damiao::ControlMode::PositionVelocity;
    double pmax_rad = 0.0;
    double vmax_rad_s = 0.0;
    double tmax_nm = 0.0;
    std::uint32_t firmware_version = 0;
    std::uint8_t raw_status = 0;
    double output_position_rad = 0.0;
    bool operable = false;
    bool drivable = false;
    std::string inoperable_reason;
};

struct ScanResult
{
    damiao::Status status;
    std::vector<DiscoveredMotor> motors;
};

// 在指定 CAN 接口上探测 ESC_ID 范围并读取基础寄存器。
ScanResult scan_motors(const ToolConfig& config,
    std::unique_ptr<damiao::ICanTransport> transport = std::make_unique<damiao::SocketCanTransport>());

const char* control_mode_name(damiao::ControlMode mode) noexcept;

}  // namespace damiao_tools
