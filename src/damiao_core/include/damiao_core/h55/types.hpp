#pragma once
#include <damiao_core/types.hpp>
#include <array>

namespace damiao::h55
{
// [H55专用] 电机坐标速度；轮方向和轮径由底盘适配层处理。
struct MotorConfig
{
    MotorAddress address;
    MappingLimits mapping{12.566, 100.0, 40.0};
    double maximum_speed = 1.0;
    std::uint32_t timeout_raw = 4000;
    std::uint32_t firmware = 0x30323936;
    std::uint32_t sub_version = 0;
};
struct BusConfig
{
    std::array<MotorConfig, 2> motors;
    std::chrono::milliseconds reply_timeout{100};
    std::chrono::milliseconds feedback_timeout{50};
    double readback_tolerance = 1e-4;
    double min_bus_voltage = 22, max_bus_voltage = 26, max_start_temperature = 45;
};
} // namespace damiao::h55
