#include <robot_wheel_control/configuration.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
namespace robot_wheel_control
{
namespace
{
void keys(const YAML::Node &n, std::initializer_list<const char *> allowed)
{
    if (!n.IsMap())
    {
        throw std::runtime_error("Expected YAML mapping");
    }
    std::set<std::string> names;
    for (auto name : allowed)
    {
        names.insert(name);
    }
    std::set<std::string> seen;
    for (const auto &item : n)
    {
        const auto key = item.first.as<std::string>();
        if (!names.count(key) || !seen.insert(key).second)
        {
            throw std::runtime_error("Unknown/duplicate YAML key: " + key);
        }
    }
    for (auto name : allowed)
    {
        if (!n[name].IsDefined())
        {
            throw std::runtime_error(std::string("Missing YAML key: ") + name);
        }
    }
}
double number(const YAML::Node &n, const char *key, bool nullable = false)
{
    if (nullable && n[key].IsNull())
    {
        return 0;
    }
    const double value = n[key].as<double>();
    if (!std::isfinite(value) || value <= 0)
    {
        throw std::runtime_error(std::string("Expected finite positive ") + key);
    }
    return value;
}
int integer(const YAML::Node &n, const char *key, int low, int high)
{
    const int v = n[key].as<int>();
    if (v < low || v > high)
    {
        throw std::runtime_error(std::string("Out of range ") + key);
    }
    return v;
}
void require(bool ok, const char *message)
{
    if (!ok)
    {
        throw std::runtime_error(message);
    }
}
} // namespace
Configuration load_configuration(const std::string &path, const std::string &backend, const std::string &mode)
{
    const auto root = YAML::LoadFile(path);
    keys(root, {"schema_version", "robot_wheel_control"});
    require(root["schema_version"].as<int>() == 1, "Unsupported schema_version");
    auto c = root["robot_wheel_control"];
    keys(c, {"namespace", "backend", "operation_mode", "commissioning", "transport", "protocol",
             "address_policy", "wheels", "geometry", "timing", "driver_protection", "limits", "position",
             "stopping", "execution", "activation", "relative_motion", "operator", "bench"});
    Configuration o;
    o.backend = backend.empty() ? c["backend"].as<std::string>() : backend;
    o.mode = mode.empty() ? c["operation_mode"].as<std::string>() : mode;
    o.name_space = c["namespace"].as<std::string>();
    require(o.name_space == "/base", "Current ROS contract uses namespace /base");
    require(o.backend == "direct_usb_sdk" || o.backend == "socketcan",
            "Unsupported backend (MCU not implemented)");
    require(o.mode == "bench" || o.mode == "base" || o.mode == "relative" || o.mode == "commissioning",
            "operation_mode must be bench, commissioning, base or relative");
    auto p = c["protocol"];
    keys(p, {"profile", "control_mode", "register_read_dlc", "expected_pmax_rad", "expected_vmax_rad_s",
             "expected_tmax_nm", "readback_relative_tolerance"});
    require(p["profile"].as<std::string>() == "h55_velocity_v1" && p["control_mode"].as<int>() == 3 &&
                p["register_read_dlc"].as<int>() == 8,
            "Only verified H55 velocity/DLC8 profile supported");
    o.bus.readback_tolerance = number(p, "readback_relative_tolerance");
    require(o.bus.readback_tolerance <= 0.001, "Readback tolerance too large");
    auto trans = c["transport"];
    keys(trans, {"can_bitrate_bps", "frame_format", "usb", "socketcan", "lock_directory"});
    require(trans["can_bitrate_bps"].as<int>() == 1000000 &&
                trans["frame_format"].as<std::string>() == "classic_standard",
            "Only Classic CAN1M supported");
    o.transport.lock_directory = trans["lock_directory"].as<std::string>();
    auto usb = trans["usb"];
    keys(usb, {"serial_number", "channel", "sdk_library_path", "sample_point"});
    o.sdk.serial_number = usb["serial_number"].as<std::string>();
    o.sdk.channel = integer(usb, "channel", 0, 0);
    o.sdk.library_path = usb["sdk_library_path"].as<std::string>();
    o.sdk.sample_point = number(usb, "sample_point");
    require(o.sdk.sample_point < 1, "Invalid sample point");
    keys(trans["socketcan"], {"interface"});
    o.transport.interface_name = trans["socketcan"]["interface"].as<std::string>();
    if (o.backend == "direct_usb_sdk")
    {
        require(!o.sdk.serial_number.empty() && !o.sdk.library_path.empty(),
                "Fill USB serial_number and sdk_library_path before real launch");
    }
    if (o.backend == "socketcan")
    {
        require(!o.transport.interface_name.empty(), "Fill SocketCAN interface");
    }
    auto flags = c["commissioning"];
    keys(flags, {"address_migration_verified", "continuous_position_verified", "loaded_stop_verified",
                 "geometry_calibrated", "max_wheel_speed_rad_s", "max_action_distance_m", "stop_margin_m", "raw_position_margin_rad"});
    o.continuous_verified = flags["continuous_position_verified"].as<bool>();
    require(flags["address_migration_verified"].as<bool>(),
            "Migrate/verify wheel addresses before real launch");
    auto policy = c["address_policy"];
    keys(policy, {"reserved_arm_esc_ids", "reserved_arm_master_ids"});
    std::set<int> used;
    for (const auto &x : policy["reserved_arm_esc_ids"])
    {
        const int id = x.as<int>();
        require(id >= 1 && id <= 15, "Invalid arm reserved ESC ID");
        used.insert(id);
        used.insert(0x100 + id);
        used.insert(0x200 + id);
        used.insert(0x300 + id);
    }
    for (const auto &x : policy["reserved_arm_master_ids"])
    {
        used.insert(x.as<int>());
    }
    used.insert(0x7ff);
    auto protection = c["driver_protection"];
    keys(protection, {"timeout_register_raw", "max_speed_rad_s", "min_bus_voltage_v", "max_bus_voltage_v",
                      "write_on_startup", "save_to_flash_on_startup"});
    require(!protection["save_to_flash_on_startup"].as<bool>(), "Automatic Flash saves are forbidden");
    o.bus.write_protection_on_startup = protection["write_on_startup"].as<bool>();
    const int timeout = integer(protection, "timeout_register_raw", 1, 20000);
    o.bus.min_bus_voltage = number(protection, "min_bus_voltage_v");
    o.bus.max_bus_voltage = number(protection, "max_bus_voltage_v");
    require(o.bus.min_bus_voltage >= 20 && o.bus.max_bus_voltage <= 65 &&
                o.bus.min_bus_voltage < o.bus.max_bus_voltage,
            "Invalid H55 supply range");
    keys(c["wheels"], {"left", "right"});
    std::size_t i = 0;
    for (auto side : {"left", "right"})
    {
        const auto w = c["wheels"][side];
        keys(w, {"joint_name", "can_id", "master_id", "direction", "extra_reduction", "effective_radius_m",
                 "expected_firmware", "expected_sub_version"});
        auto &m = o.bus.motors[i];
        m.address.esc_id = integer(w, "can_id", 1, 15);
        m.address.mst_id = integer(w, "master_id", 1, 0x7fe);
        require(!used.count(m.address.esc_id) && !used.count(m.address.mst_id) &&
                    m.address.esc_id != m.address.mst_id,
                "Wheel/arm CAN address collision");
        for (const auto id : {int(m.address.esc_id), int(m.address.mst_id), 0x200 + int(m.address.esc_id)})
        {
            require(used.insert(id).second, "CAN frame address collision");
        }
        o.joint_names[i] = w["joint_name"].as<std::string>();
        require(!o.joint_names[i].empty(), "Empty joint name");
        o.direction[i] = w["direction"].as<int>();
        require(o.direction[i] == 1 || o.direction[i] == -1, "Direction must be -1 or +1");
        o.reduction[i] = number(w, "extra_reduction");
        o.radius[i] = number(w, "effective_radius_m", true);
        auto fw = w["expected_firmware"].as<std::string>();
        require(fw.size() == 4, "Firmware must be four ASCII bytes");
        m.firmware = 0;
        for (int j = 0; j < 4; ++j)
        {
            m.firmware |= std::uint32_t(std::uint8_t(fw[j])) << (8 * j);
        }
        m.sub_version = integer(w, "expected_sub_version", 0, 65535);
        m.mapping = {number(p, "expected_pmax_rad"), number(p, "expected_vmax_rad_s"),
                     number(p, "expected_tmax_nm")};
        m.maximum_speed = number(protection, "max_speed_rad_s");
        m.timeout_raw = timeout;
        ++i;
    }
    require(o.joint_names[0] != o.joint_names[1], "Duplicate wheel joint name");
    auto g = c["geometry"];
    keys(g, {"wheel_separation_m", "base_frame_id", "odom_frame_id"});
    require(g["base_frame_id"].as<std::string>() == "base_link" &&
                g["odom_frame_id"].as<std::string>() == "odom",
            "Current frame contract is odom/base_link");
    o.separation = number(g, "wheel_separation_m", true);
    auto t = c["timing"];
    keys(t, {"control_rate_hz", "odom_publish_rate_hz", "status_publish_rate_hz", "command_timeout_ms",
             "feedback_timeout_ms", "max_control_lateness_ms", "management_timeout_ms",
             "stop_confirmation_timeout_ms", "idle_poll_rate_hz", "idle_feedback_timeout_ms"});
    o.control_hz = number(t, "control_rate_hz");
    o.status_hz = number(t, "status_publish_rate_hz");
    // 失能待机与运动分别约束；待机容错不能放宽运动反馈保护。
    o.idle_poll_hz = number(t, "idle_poll_rate_hz");
    o.idle_feedback_timeout_ms = integer(t, "idle_feedback_timeout_ms", 100, 1000);
    require(o.idle_poll_hz >= 20 && o.idle_poll_hz <= o.control_hz &&
                o.idle_feedback_timeout_ms >= 3000.0 / o.idle_poll_hz,
            "Invalid disabled polling timing");
    require(o.control_hz >= 20 && o.control_hz <= 200 && o.status_hz <= o.control_hz &&
                number(t, "odom_publish_rate_hz") <= o.control_hz,
            "Invalid control/publish frequency");
    o.command_timeout_ms = integer(t, "command_timeout_ms", 20, 500);
    o.lateness_ms = integer(t, "max_control_lateness_ms", 10, 100);
    o.bus.feedback_timeout = std::chrono::milliseconds(integer(t, "feedback_timeout_ms", 10, 100));
    o.bus.reply_timeout = std::chrono::milliseconds(integer(t, "management_timeout_ms", 10, 200));
    o.stop_timeout_ms = integer(t, "stop_confirmation_timeout_ms", 100, 5000);
    require(o.command_timeout_ms < timeout * 0.05 && o.bus.feedback_timeout.count() < timeout * 0.05,
            "Host timeouts must precede driver TIMEOUT");
    auto l = c["limits"];
    keys(l, {"max_wheel_speed_rad_s", "max_wheel_acceleration_rad_s2", "max_wheel_deceleration_rad_s2", "bench_max_travel_rad",
             "max_linear_speed_m_s", "max_angular_speed_rad_s", "max_linear_acceleration_m_s2",
             "max_angular_acceleration_rad_s2", "max_linear_deceleration_m_s2", "max_angular_deceleration_rad_s2", "max_reported_torque_nm", "max_motor_temperature_c",
             "max_driver_temperature_c"});
    o.wheel_speed = number(l, "max_wheel_speed_rad_s");
    o.wheel_acceleration = number(l, "max_wheel_acceleration_rad_s2");
    o.wheel_deceleration = number(l, "max_wheel_deceleration_rad_s2");
    o.bench_travel = number(l, "bench_max_travel_rad");
    for (std::size_t k = 0; k < 2; ++k)
    {
        require(o.wheel_speed * o.reduction[k] <= o.bus.motors[k].maximum_speed,
                "Software wheel limit exceeds driver limit");
    }
    keys(c["bench"], {"max_wheel_speed_rad_s"});
    if (o.mode == "bench")
    {
        o.wheel_speed = std::min(o.wheel_speed, number(c["bench"], "max_wheel_speed_rad_s"));
    }
    o.raw_position_margin = number(flags, "raw_position_margin_rad");
    if (o.mode == "commissioning")
    {
        o.wheel_speed = std::min(o.wheel_speed, number(flags, "max_wheel_speed_rad_s"));
        o.action_distance = number(flags, "max_action_distance_m");
        o.stop_margin = number(flags, "stop_margin_m");
        require(o.stop_margin < o.action_distance && o.radius[0] > 0 && o.radius[1] > 0 && o.separation > 0,
                "Commissioning needs nominal geometry and a positive usable action budget");
    }
    o.linear_speed = number(l, "max_linear_speed_m_s", true);
    o.angular_speed = number(l, "max_angular_speed_rad_s", true);
    o.linear_acceleration = number(l, "max_linear_acceleration_m_s2", true);
    o.linear_deceleration = number(l, "max_linear_deceleration_m_s2", true);
    o.angular_acceleration = number(l, "max_angular_acceleration_rad_s2", true);
    o.angular_deceleration = number(l, "max_angular_deceleration_rad_s2", true);
    if (o.mode != "bench")
    {
        require(o.linear_speed > 0 && o.angular_speed > 0 && o.linear_acceleration > 0 &&
                    o.linear_deceleration > 0 && o.angular_acceleration > 0 && o.angular_deceleration > 0,
                "Ground modes need all body velocity/acceleration/deceleration limits");
    }
    auto op = c["operator"];
    keys(op, {"default_wheel_speed_rad_s", "default_linear_speed_m_s", "default_angular_speed_rad_s",
              "default_duration_s", "max_duration_s", "command_publish_rate_hz", "management_wait_s"});
    o.default_speed = number(op, "default_wheel_speed_rad_s");
    o.default_linear = number(op, "default_linear_speed_m_s");
    o.default_angular = number(op, "default_angular_speed_rad_s");
    o.default_duration = number(op, "default_duration_s");
    o.max_duration = number(op, "max_duration_s", true);
    o.command_hz = number(op, "command_publish_rate_hz");
    o.management_wait = number(op, "management_wait_s");
    require(o.command_hz <= o.control_hz && 2000.0 / o.command_hz < o.command_timeout_ms &&
                (!o.max_duration || o.default_duration <= o.max_duration), "Invalid operator timing");
    o.torque_limit = number(l, "max_reported_torque_nm");
    o.driver_temperature = number(l, "max_driver_temperature_c");
    o.motor_temperature = number(l, "max_motor_temperature_c");
    o.bus.max_start_temperature = std::min(o.motor_temperature, o.driver_temperature);
    for (auto name : {"max_linear_speed_m_s", "max_angular_speed_rad_s", "max_linear_acceleration_m_s2",
                      "max_angular_acceleration_rad_s2"})
    {
        number(l, name, true);
    }
    auto pos = c["position"];
    keys(pos, {"source", "wrap_period_rad", "max_unwrap_gap_ms"});
    require(pos["source"].as<std::string>() == "feedback_unwrap", "Unsupported position source");
    o.wrap_period = number(pos, "wrap_period_rad", true);
    o.unwrap_gap_ms = integer(pos, "max_unwrap_gap_ms", 10, 100);
    auto s = c["stopping"];
    keys(s, {"standstill_velocity_rad_s", "standstill_position_window_rad", "standstill_hold_ms",
             "normal_stop_behavior", "require_parking_confirmation_for_arm"});
    o.standstill_speed = number(s, "standstill_velocity_rad_s", true);
    o.standstill_position = number(s, "standstill_position_window_rad", true);
    o.standstill_ms = integer(s, "standstill_hold_ms", 100, 5000);
    require(s["normal_stop_behavior"].as<std::string>() == "zero_then_disable" &&
                s["require_parking_confirmation_for_arm"].as<bool>(),
            "Unsupported stop/parking policy");
    auto e = c["execution"];
    keys(e, {"executor_threads", "management_queue_depth", "completed_request_history_depth",
             "feedback_queue_depth", "command_mailbox_depth"});
    o.executor_threads = integer(e, "executor_threads", 2, 8);
    o.management_capacity = integer(e, "management_queue_depth", 1, 16);
    o.history_capacity = integer(e, "completed_request_history_depth", 16, 1024);
    o.sdk.receive_capacity = integer(e, "feedback_queue_depth", 16, 4096);
    require(e["command_mailbox_depth"].as<int>() == 1, "Only latest-target mailbox is supported");
    auto a = c["activation"];
    keys(a,
         {"allow_enable_on_activate", "auto_reenable_after_reconnect", "require_new_command_after_enable"});
    require(!a["allow_enable_on_activate"].as<bool>() && !a["auto_reenable_after_reconnect"].as<bool>() &&
                a["require_new_command_after_enable"].as<bool>(),
            "Automatic enable/replay is forbidden");
    auto relative = c["relative_motion"];
    keys(relative, {"position_gain", "synchronization_gain", "wheel_position_tolerance_rad",
                    "settled_velocity_rad_s", "settle_hold_ms", "heartbeat_timeout_ms", "stall_timeout_s",
                    "progress_step_rad", "max_distance_m", "max_yaw_rad", "max_timeout_s", "timeout_factor", "timeout_margin_s"});
    o.relative.position_gain = number(relative, "position_gain");
    o.relative.sync_gain = number(relative, "synchronization_gain");
    o.relative.wheel_tolerance = number(relative, "wheel_position_tolerance_rad");
    o.relative.settled_speed = number(relative, "settled_velocity_rad_s");
    o.relative.settle_ms = integer(relative, "settle_hold_ms", 200, 2000);
    o.relative.heartbeat_ms = integer(relative, "heartbeat_timeout_ms", 100, 1000);
    o.relative.stall_timeout_s = number(relative, "stall_timeout_s");
    o.relative.progress_step = number(relative, "progress_step_rad");
    o.relative.max_distance = number(relative, "max_distance_m", true);
    o.relative.max_yaw = number(relative, "max_yaw_rad");
    o.relative.max_timeout_s = number(relative, "max_timeout_s", true);
    o.relative.timeout_factor = number(relative, "timeout_factor");
    o.relative.timeout_margin = number(relative, "timeout_margin_s");
    require(2000.0 / o.command_hz < o.relative.heartbeat_ms,
            "Operator rate cannot maintain relative heartbeat");
    for (const auto &motor : o.bus.motors)
    {
        require(o.raw_position_margin < motor.mapping.position_rad &&
                    motor.maximum_speed <= motor.mapping.velocity_rad_s,
                "Protection velocity or raw position margin exceeds feedback mapping");
    }
    require(o.relative.progress_step < o.relative.wheel_tolerance && o.relative.timeout_factor >= 1 &&
                (!o.relative.max_timeout_s || o.relative.max_timeout_s > o.relative.stall_timeout_s),
            "Invalid relative motion limits");
    if (o.mode == "base" || o.mode == "relative")
    {
        require(o.radius[0] > 0 && o.radius[1] > 0 && o.separation > 0 && o.continuous_verified &&
                    o.wrap_period > 0,
                "Base mode requires calibrated geometry and verified position wrap");
        require(1000.0 / o.idle_poll_hz + 2000.0 / o.control_hz < o.unwrap_gap_ms,
                "Raise idle_poll_rate_hz to leave position continuity scheduling margin");
        // 实车几何、停车与连续位置均须显式验收。
        {
            require(flags["geometry_calibrated"].as<bool>() && flags["loaded_stop_verified"].as<bool>() &&
                        o.standstill_speed > 0 && o.standstill_position > 0,
                    "Complete loaded stop and geometry commissioning for base mode");
            for (auto name : {"max_linear_speed_m_s", "max_angular_speed_rad_s",
                              "max_linear_acceleration_m_s2", "max_angular_acceleration_rad_s2"})
            {
                number(l, name);
            }
        }
    }
    // 可复现FNV-1a摘要用于区分配置会话，不用于签名或安全认证。
    std::ifstream file(path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::uint64_t hash = 14695981039346656037ULL;
    for (unsigned char ch : buffer.str() + o.backend + o.mode)
    {
        hash = (hash ^ ch) * 1099511628211ULL;
    }
    std::ostringstream digest;
    digest << std::hex << hash;
    o.digest = digest.str();
    o.source_path = path;
    // Return the effective profile limit, not an inactive higher global setting.
    auto effective = YAML::Clone(root);
    effective["robot_wheel_control"]["operation_mode"] = o.mode;
    effective["robot_wheel_control"]["backend"] = o.backend;
    effective["robot_wheel_control"]["limits"]["max_wheel_speed_rad_s"] = o.wheel_speed;
    o.yaml = YAML::Dump(effective);
    return o;
}
} // namespace robot_wheel_control
