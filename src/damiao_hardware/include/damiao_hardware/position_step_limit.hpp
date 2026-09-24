#pragma once

#include <algorithm>

namespace damiao_hardware
{

// 控制器重规划可能让目标跳变；按实际发送间隔限制单周期关节位移。
inline double limit_position_step(double requested, double previous,
    double max_velocity_rad_s, double interval_seconds)
{
    constexpr double speed_margin = 0.9;
    const double max_step = max_velocity_rad_s * interval_seconds * speed_margin;
    return std::clamp(requested, previous - max_step, previous + max_step);
}

}  // namespace damiao_hardware
