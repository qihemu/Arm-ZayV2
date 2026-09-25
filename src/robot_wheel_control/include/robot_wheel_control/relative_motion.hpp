#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace robot_wheel_control
{
// 相对任务保持速度模式；目标来自轮编码器增量，不用运行时间推算位移。
struct RelativeGoal
{
    std::uint8_t kind = 0; // 1距离m，2车体转角rad，3悬空双轮逻辑转角rad。
    double value = 0, max_speed = 0, timeout_s = 0;
};
struct RelativeSettings
{
    double position_gain = 2.0, sync_gain = 1.0;
    double wheel_tolerance = 0.01, settled_speed = 0.08;
    double max_distance = 1.0, max_yaw = 3.141592653589793;
    double stall_timeout_s = 4.0, progress_step = 0.002;
    int settle_ms = 500, heartbeat_ms = 500;
    double max_timeout_s = 180;
};
// 独立纯计算模块：有比例减速、制动距离约束和左右归一化进度同步。
inline std::array<double, 2> relative_velocity(const std::array<double, 2> &target,
                                               const std::array<double, 2> &actual,
                                               const std::array<double, 2> &caps, double acceleration,
                                               const RelativeSettings &settings)
{
    std::array<double, 2> speed{};
    const double p0 = actual[0] / target[0], p1 = actual[1] / target[1];
    for (std::size_t i = 0; i < 2; ++i)
    {
        const double error = target[i] - actual[i];
        // 失能前收敛到最终容差的一半，为释放驱动后的微小变化留出余量。
        const double approach_tolerance = settings.wheel_tolerance * 0.5;
        if (std::abs(error) <= approach_tolerance)
        {
            continue;
        }
        const double sync = settings.sync_gain * target[i] * (i == 0 ? p1 - p0 : p0 - p1);
        const double desired = settings.position_gain * error + sync;
        const double braking =
            std::sqrt(2 * acceleration * std::max(0.0, std::abs(error) - approach_tolerance));
        const double cap = std::min(caps[i], braking);
        // 同步校正不能把本轮推向目标的反方向；已超调时允许小速度回退。
        speed[i] = std::copysign(std::clamp(desired * std::copysign(1.0, error), 0.0, cap), error);
    }
    return speed;
}
} // namespace robot_wheel_control
