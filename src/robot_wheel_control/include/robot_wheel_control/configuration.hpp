#pragma once
#include <damiao_core/h55/types.hpp>
#include <damiao_core/transports/dm_usb_sdk_transport.hpp>
#include <array>
#include <string>
#include <robot_wheel_control/relative_motion.hpp>

namespace robot_wheel_control
{
// 配置一次加载后不可热改；几何参数必须来自实机标定。
struct Configuration
{
    std::string backend, mode, name_space, digest;
    damiao::h55::BusConfig bus;
    damiao::TransportConfig transport;
    damiao::DmUsbSdkConfig sdk;
    std::array<std::string, 2> joint_names;
    std::array<int, 2> direction;
    std::array<double, 2> reduction, radius;
    double separation = 0, wrap_period = 0;
    double wheel_speed = 0.2, wheel_acceleration = 0.4, bench_travel = 1;
    double torque_limit = 0.8, driver_temperature = 45, motor_temperature = 45;
    double control_hz = 100, status_hz = 10;
    double idle_poll_hz = 25;
    int idle_feedback_timeout_ms = 250;
    double standstill_speed = 0, standstill_position = 0;
    int command_timeout_ms = 150, lateness_ms = 30, stop_timeout_ms = 1000, standstill_ms = 500;
    int unwrap_gap_ms = 50, executor_threads = 3, management_capacity = 4, history_capacity = 128;
    bool continuous_verified = false;
    RelativeSettings relative;
};
Configuration load_configuration(const std::string &path, const std::string &backend_override = "",
                                 const std::string &mode_override = "");
} // namespace robot_wheel_control
