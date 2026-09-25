#include <damiao_hardware/damiao_system_hardware.hpp>
#include <damiao_hardware/position_step_limit.hpp>

#include <damiao_core/protocol.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <system_error>
#include <unordered_set>
#include <vector>

namespace damiao_hardware
{
namespace
{

// 严格解析 URDF 参数；缺失、尾随字符和非有限数均不能进入运行配置。
const std::string& required(const std::unordered_map<std::string, std::string>& values,
    const std::string& key)
{
    const auto found = values.find(key);
    if (found == values.end() || found->second.empty())
    {
        throw std::invalid_argument("Missing parameter: " + key);
    }
    return found->second;
}

double finite_double(const std::unordered_map<std::string, std::string>& values,
    const std::string& key)
{
    const auto& value = required(values, key);
    std::size_t used = 0;
    const double parsed = std::stod(value, &used);
    if (used != value.size() || !std::isfinite(parsed))
    {
        throw std::invalid_argument("Invalid finite number: " + key);
    }
    return parsed;
}

int whole_number(const std::unordered_map<std::string, std::string>& values,
    const std::string& key)
{
    const auto& value = required(values, key);
    std::size_t used = 0;
    const int parsed = std::stoi(value, &used);
    if (used != value.size())
    {
        throw std::invalid_argument("Invalid integer: " + key);
    }
    return parsed;
}

std::chrono::milliseconds positive_milliseconds(
    const std::unordered_map<std::string, std::string>& values, const std::string& key)
{
    const int count = whole_number(values, key);
    if (count <= 0 || count > 1000)
    {
        throw std::invalid_argument("Expected 1..1000 milliseconds: " + key);
    }
    return std::chrono::milliseconds(count);
}

bool is_only(const std::vector<hardware_interface::InterfaceInfo>& interfaces,
    std::initializer_list<const char*> names)
{
    if (interfaces.size() != names.size())
    {
        return false;
    }
    for (const char* name : names)
    {
        if (std::count_if(interfaces.begin(), interfaces.end(),
            [name](const auto& item) { return item.name == name; }) != 1)
        {
            return false;
        }
    }
    return true;
}

}  // namespace

DamiaoSystemHardware::~DamiaoSystemHardware()
{
    close_bus();
}

hardware_interface::CallbackReturn DamiaoSystemHardware::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    try
    {
        if (info.name.empty() || info.joints.empty() || info.joints.size() > damiao::max_motors)
        {
            throw std::invalid_argument("Expected one to six joints.");
        }
        axis_count_ = info.joints.size();
        const auto& hardware = info.hardware_parameters;
        config_.can_interface = required(hardware, "can_interface");
        config_.feedback_timeout = positive_milliseconds(hardware, "feedback_timeout_ms");
        config_.management_timeout = positive_milliseconds(hardware, "management_timeout_ms");
        config_.management_quiet_period = positive_milliseconds(hardware, "management_quiet_period_ms");
        config_.inactive_poll_period = positive_milliseconds(hardware, "inactive_poll_period_ms");
        config_.max_control_period = positive_milliseconds(hardware, "max_control_period_ms");
        const auto allow = hardware.find("allow_enable_on_activate");
        if (allow != hardware.end())
        {
            if (allow->second != "true" && allow->second != "false")
            {
                throw std::invalid_argument("allow_enable_on_activate must be true or false.");
            }
            config_.allow_enable_on_activate = allow->second == "true";
        }
        if (config_.management_quiet_period >= config_.management_timeout
            || config_.inactive_poll_period
                + axis_count_ * config_.management_timeout >= config_.feedback_timeout)
        {
            throw std::invalid_argument("Feedback timeout cannot cover a complete inactive polling pass.");
        }

        std::unordered_set<std::string> joint_names;
        std::unordered_set<std::string> motor_names;
        std::unordered_set<int> esc_ids;
        std::unordered_set<int> mst_ids;
        for (std::size_t index = 0; index < axis_count_; ++index)
        {
            const auto& joint = info.joints[index];
            if (joint.name.empty()
                || !is_only(joint.command_interfaces, {hardware_interface::HW_IF_POSITION})
                || !is_only(joint.state_interfaces,
                    {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY}))
            {
                throw std::invalid_argument("Each joint needs position command and position/velocity state.");
            }
            auto& axis = axes_[index];
            axis.joint_name = joint.name;
            axis.motor_name = required(joint.parameters, "motor_name");
            // 保留旧单轴 URDF 的硬件级型号/ID 参数；多轴参数位于各 joint。
            const auto& motor_params = axis_count_ == 1 && joint.parameters.count("model") == 0
                ? hardware : joint.parameters;
            axis.model = required(motor_params, "model");
            const int esc = whole_number(motor_params, "esc_id");
            const int mst = whole_number(motor_params, "mst_id");
            if (esc < 1 || esc > 15 || mst < 1 || mst > 2047)
            {
                throw std::invalid_argument("ESC_ID or MST_ID is outside the supported range.");
            }
            axis.esc_id = static_cast<std::uint16_t>(esc);
            axis.mst_id = static_cast<std::uint16_t>(mst);
            if (damiao::DamiaoProtocol::validate_address({axis.esc_id, axis.mst_id}).code
                != damiao::ErrorCode::Ok || !joint_names.insert(axis.joint_name).second
                || !motor_names.insert(axis.motor_name).second || !esc_ids.insert(esc).second
                || !mst_ids.insert(mst).second)
            {
                throw std::invalid_argument("Duplicate or conflicting joint/motor name or CAN address.");
            }
            if (axis_count_ == 1 && hardware.count("motor_name") != 0
                && required(hardware, "motor_name") != axis.motor_name)
            {
                throw std::invalid_argument("Joint motor_name does not match hardware motor_name.");
            }
            axis.direction = whole_number(joint.parameters, "direction");
            axis.zero_offset_motor_output_rad = finite_double(
                joint.parameters, "zero_offset_motor_output_rad");
            axis.extra_reduction = finite_double(joint.parameters, "extra_reduction");
            axis.min_position_rad = finite_double(joint.parameters, "min_position_rad");
            axis.max_position_rad = finite_double(joint.parameters, "max_position_rad");
            axis.max_velocity_rad_s = finite_double(joint.parameters, "max_velocity_rad_s");
            axis.activation_position_tolerance_rad = finite_double(
                joint.parameters, "activation_position_tolerance_rad");
            if ((axis.direction != 1 && axis.direction != -1) || axis.extra_reduction <= 0.0
                || axis.min_position_rad >= axis.max_position_rad || axis.max_velocity_rad_s <= 0.0
                || axis.activation_position_tolerance_rad <= 0.0
                || !std::isfinite(axis.extra_reduction * axis.max_velocity_rad_s)
                || !std::isfinite(motor_position(index, axis.min_position_rad))
                || !std::isfinite(motor_position(index, axis.max_position_rad)))
            {
                throw std::invalid_argument("Invalid conversion or joint limits: " + axis.joint_name);
            }
        }
        for (std::size_t first = 0; first < axis_count_; ++first)
        {
            for (std::size_t second = first + 1; second < axis_count_; ++second)
            {
                if ((axes_[first].mst_id & 0xFF) == axes_[second].esc_id
                    || (axes_[second].mst_id & 0xFF) == axes_[first].esc_id)
                {
                    throw std::invalid_argument("Cross-axis feedback/command CAN ID collision.");
                }
            }
        }
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"), "on_init: %s", error.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    position_state_.fill(invalid);
    velocity_state_.fill(invalid);
    position_command_.fill(invalid);
    last_command_.fill(invalid);
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DamiaoSystemHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(2 * axis_count_);
    for (std::size_t index = 0; index < axis_count_; ++index)
    {
        interfaces.emplace_back(axes_[index].joint_name, hardware_interface::HW_IF_POSITION,
            &position_state_[index]);
        interfaces.emplace_back(axes_[index].joint_name, hardware_interface::HW_IF_VELOCITY,
            &velocity_state_[index]);
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> DamiaoSystemHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(axis_count_);
    for (std::size_t index = 0; index < axis_count_; ++index)
    {
        interfaces.emplace_back(axes_[index].joint_name, hardware_interface::HW_IF_POSITION,
            &position_command_[index]);
    }
    return interfaces;
}

hardware_interface::CallbackReturn DamiaoSystemHardware::on_configure(const rclcpp_lifecycle::State&)
{
    close_bus();
    fault_ = false;
    bus_ = std::make_unique<damiao::DamiaoBus>();
    for (std::size_t index = 0; index < axis_count_; ++index)
    {
        const auto& axis = axes_[index];
        const double first = motor_position(index, axis.min_position_rad);
        const double last = motor_position(index, axis.max_position_rad);
        damiao::MotorConfig motor;
        motor.name = axis.motor_name;
        motor.model = axis.model;
        motor.address = {axis.esc_id, axis.mst_id};
        motor.mode = damiao::ControlMode::PositionVelocity;
        motor.min_output_position_rad = std::min(first, last);
        motor.max_output_position_rad = std::max(first, last);
        motor.max_output_speed_rad_s = axis.extra_reduction * axis.max_velocity_rad_s;
        const auto registration = bus_->register_motor(motor);
        if (registration.status.code != damiao::ErrorCode::Ok || registration.value != index)
        {
            RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
                "Motor registration failed for %s", axis.motor_name.c_str());
            close_bus();
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    damiao::BusConfig bus_config;
    bus_config.transport.interface_name = config_.can_interface;
    bus_config.feedback_timeout = config_.feedback_timeout;
    bus_config.management_quiet_period = config_.management_quiet_period;
    const auto opened = bus_->open(bus_config);
    if (opened.code != damiao::ErrorCode::Ok)
    {
        RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"), "Bus open: %s", opened.message.c_str());
        close_bus();
        return hardware_interface::CallbackReturn::ERROR;
    }
    for (std::size_t index = 0; index < axis_count_; ++index)
    {
        // 各电机独立读回模式和编码范围，任一失配都阻止整机进入 inactive。
        const auto synchronized = bus_->synchronize_motor(
            index, damiao::SteadyClock::now() + config_.management_timeout);
        const auto verified = bus_->motor_config(index);
        const auto& axis = axes_[index];
        const double first = motor_position(index, axis.min_position_rad);
        const double last = motor_position(index, axis.max_position_rad);
        if (synchronized.code != damiao::ErrorCode::Ok || !verified.value
            || verified.value->mode != damiao::ControlMode::PositionVelocity
            || std::max(std::abs(first), std::abs(last)) > verified.value->mapping.position_rad
            || axis.extra_reduction * axis.max_velocity_rad_s > verified.value->mapping.velocity_rad_s)
        {
            RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
                "Motor mode/mapping mismatch: %s", axis.motor_name.c_str());
            close_bus();
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    for (std::size_t index = 0; index < axis_count_; ++index)
    {
        const auto current = bus_->query_state(
            index, damiao::SteadyClock::now() + config_.management_timeout);
        double position = 0.0;
        double velocity = 0.0;
        // 分开报告通信、失能状态和反馈有效性，避免把越界但有效的反馈当作配置失败。
        if (current.status.code != damiao::ErrorCode::Ok || !current.value)
        {
            RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
                "Initial feedback query failed: %s (error=%d, detail=%s)",
                axes_[index].joint_name.c_str(), static_cast<int>(current.status.code),
                current.status.message.c_str());
            close_bus();
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (current.value->raw_status != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
                "Initial motor is not disabled: %s (raw_status=%u, %s)",
                axes_[index].joint_name.c_str(),
                static_cast<unsigned int>(current.value->raw_status),
                damiao::DamiaoProtocol::status_description(current.value->raw_status));
            close_bus();
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (!decode_state(index, *current.value, position, velocity))
        {
            const auto& axis = axes_[index];
            RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
                "Initial feedback invalid: %s "
                "(valid=%d, motor_position=%.6f rad, motor_velocity=%.6f rad/s, "
                "joint_position=%.6f rad, limits=[%.6f, %.6f] rad)",
                axis.joint_name.c_str(), current.value->valid,
                current.value->output_position_rad, current.value->output_velocity_rad_s,
                position, axis.min_position_rad, axis.max_position_rad);
            close_bus();
            return hardware_interface::CallbackReturn::ERROR;
        }
        // 失能期保留真实角度供诊断；越界只阻止后续激活，不让启动时的配置失败杀死进程。
        if (!position_within_limits(index, position))
        {
            const auto& axis = axes_[index];
            RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
                "Initial feedback outside limits; activation will be refused: %s "
                "(joint_position=%.6f rad, limits=[%.6f, %.6f] rad)",
                axis.joint_name.c_str(), position, axis.min_position_rad, axis.max_position_rad);
        }
        position_state_[index] = position;
        velocity_state_[index] = velocity;
        last_state_received_[index] = current.value->received_at;
        position_command_[index] = position;
        last_command_[index] = position;
    }
    configured_ = true;
    try
    {
        start_inactive_polling();
    }
    catch (const std::system_error& error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
            "Could not start inactive query thread: %s", error.what());
        close_bus();
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DamiaoSystemHardware::on_activate(const rclcpp_lifecycle::State&)
{
    // 多轴使能时仍为逐帧事务；默认只读，台架验证时序及停止策略后才允许激活。
    if (!configured_ || fault_ || !config_.allow_enable_on_activate)
    {
        RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
            "Activation requires configured hardware and explicit bench enable approval.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    stop_inactive_polling();
    const auto fail = [this](const char* reason)
    {
        RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"), "Activation failed: %s", reason);
        fault_ = true;
        if (enable_attempted_)
        {
            disable_all();
        }
        return hardware_interface::CallbackReturn::ERROR;
    };
    std::array<double, damiao::max_motors> before{};
    for (std::size_t index = 0; index < axis_count_; ++index)
    {
        const auto current = bus_->query_state(
            index, damiao::SteadyClock::now() + config_.management_timeout);
        double position = 0.0;
        double velocity = 0.0;
        if (current.status.code != damiao::ErrorCode::Ok || !current.value
            || current.value->raw_status != 0
            || !decode_state(index, *current.value, position, velocity)
            || !position_within_limits(index, position))
        {
            // 使能之前的检查可以重试；恢复失能期查询并保持硬件为 inactive。
            const auto& axis = axes_[index];
            RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
                "Activation precheck failed for %s: joint_position=%.6f rad, "
                "limits=[%.6f, %.6f] rad, feedback_error=%d, raw_status=%u",
                axis.joint_name.c_str(), position, axis.min_position_rad, axis.max_position_rad,
                static_cast<int>(current.status.code),
                current.value ? static_cast<unsigned>(current.value->raw_status) : 0U);
            try
            {
                start_inactive_polling();
            }
            catch (const std::system_error& error)
            {
                return fail(error.what());
            }
            return hardware_interface::CallbackReturn::FAILURE;
        }
        before[index] = position;
        position_state_[index] = position;
        velocity_state_[index] = velocity;
        last_state_received_[index] = current.value->received_at;
        position_command_[index] = position;
        last_command_[index] = position;
    }
    enable_attempted_ = true;
    for (std::size_t index = 0; index < axis_count_; ++index)
    {
        const auto enabled = bus_->enable(
            index, damiao::SteadyClock::now() + config_.management_timeout);
        const auto after = bus_->snapshot(index);
        double position = 0.0;
        double velocity = 0.0;
        if (enabled.code != damiao::ErrorCode::Ok || after.status.code != damiao::ErrorCode::Ok
            || !after.value || after.value->raw_status != 1
            || !decode_state(index, *after.value, position, velocity)
            || !position_within_limits(index, position)
            || std::abs(position - before[index]) > axes_[index].activation_position_tolerance_rad)
        {
            return fail("Enable confirmation or initial hold position check failed.");
        }
        position_state_[index] = position;
        velocity_state_[index] = velocity;
        last_state_received_[index] = after.value->received_at;
        position_command_[index] = position;
        last_command_[index] = position;
    }
    // 六台均使能后再次核对整组反馈，避免只依赖逐台使能时的旧状态。
    for (std::size_t index = 0; index < axis_count_; ++index)
    {
        const auto current = bus_->query_state(
            index, damiao::SteadyClock::now() + config_.management_timeout);
        double position = 0.0;
        double velocity = 0.0;
        if (current.status.code != damiao::ErrorCode::Ok || !current.value
            || current.value->raw_status != 1
            || !decode_state(index, *current.value, position, velocity)
            || !position_within_limits(index, position)
            || std::abs(position - before[index]) > axes_[index].activation_position_tolerance_rad)
        {
            return fail("Post-enable all-axis feedback check failed.");
        }
        position_state_[index] = position;
        velocity_state_[index] = velocity;
        last_state_received_[index] = current.value->received_at;
        position_command_[index] = position;
        last_command_[index] = position;
    }
    const auto started = bus_->begin_control();
    if (started.code != damiao::ErrorCode::Ok)
    {
        return fail("Some enabled feedback expired before batch control began.");
    }
    std::array<damiao::PositionVelocityCommand, damiao::max_motors> hold{};
    std::array<damiao::ErrorCode, damiao::max_motors> results{};
    for (std::size_t index = 0; index < axis_count_; ++index)
    {
        hold[index] = {motor_position(index, position_state_[index]),
            axes_[index].extra_reduction * axes_[index].max_velocity_rad_s};
    }
    const auto hold_sent_at = damiao::SteadyClock::now();
    if (bus_->send_position_velocity_batch(hold.data(), axis_count_, results.data())
        != damiao::ErrorCode::Ok)
    {
        return fail("Initial all-axis hold batch failed.");
    }
    last_command_sent_at_ = hold_sent_at;
    last_rate_warning_at_ = {};
    initial_hold_pending_ = true;
    active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DamiaoSystemHardware::on_deactivate(const rclcpp_lifecycle::State&)
{
    // 停止整组命令后逐轴失能；此策略仅适用于有外部支撑的台架。
    if (!bus_)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    if (!disable_all())
    {
        fault_ = true;
        return hardware_interface::CallbackReturn::ERROR;
    }
    active_ = false;
    last_command_sent_at_ = {};
    last_rate_warning_at_ = {};
    initial_hold_pending_ = false;
    for (std::size_t index = 0; index < axis_count_; ++index)
    {
        position_command_[index] = position_state_[index];
        last_command_[index] = position_state_[index];
    }
    try
    {
        start_inactive_polling();
    }
    catch (const std::system_error& error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
            "Could not restart inactive query thread: %s", error.what());
        fault_ = true;
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DamiaoSystemHardware::on_cleanup(const rclcpp_lifecycle::State&)
{
    close_bus();
    fault_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DamiaoSystemHardware::on_shutdown(const rclcpp_lifecycle::State&)
{
    close_bus();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DamiaoSystemHardware::on_error(const rclcpp_lifecycle::State&)
{
    fault_ = true;
    close_bus();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DamiaoSystemHardware::read(const rclcpp::Time&, const rclcpp::Duration&)
{
    if (!configured_ || !bus_ || fault_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
            "Read rejected: configured=%d bus=%d fault=%d",
            configured_, static_cast<bool>(bus_), fault_);
        return hardware_interface::return_type::ERROR;
    }
    std::array<damiao::MotorState, damiao::max_motors> states{};
    const auto status = bus_->snapshot_into(states.data(), axis_count_);
    if (status == damiao::ErrorCode::WouldBlock)
    {
        const auto now = damiao::SteadyClock::now();
        bool all_recent = true;
        for (std::size_t index = 0; index < axis_count_; ++index)
        {
            all_recent = all_recent
                && now - last_state_received_[index] <= config_.feedback_timeout;
        }
        if (all_recent)
        {
            return hardware_interface::return_type::OK;
        }
    }
    std::array<double, damiao::max_motors> positions{};
    std::array<double, damiao::max_motors> velocities{};
    if (status != damiao::ErrorCode::Ok)
    {
        RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
            "Read failed: CAN feedback snapshot error=%d", static_cast<int>(status));
        fault_ = true;
        return hardware_interface::return_type::ERROR;
    }
    // 先检查全部反馈，避免前轴状态已更新而后轴反馈无效。
    for (std::size_t index = 0; index < axis_count_; ++index)
    {
        if (states[index].raw_status != (active_ ? 1 : 0)
            || !decode_state(index, states[index], positions[index], velocities[index])
            || (active_ && !position_within_limits(index, positions[index])))
        {
            RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
                "Read failed for %s: valid=%d raw_status=%u expected=%u position=%.6f velocity=%.6f",
                axes_[index].joint_name.c_str(), states[index].valid,
                static_cast<unsigned>(states[index].raw_status), active_ ? 1U : 0U,
                states[index].output_position_rad, states[index].output_velocity_rad_s);
            fault_ = true;
            return hardware_interface::return_type::ERROR;
        }
    }
    for (std::size_t index = 0; index < axis_count_; ++index)
    {
        position_state_[index] = positions[index];
        velocity_state_[index] = velocities[index];
        last_state_received_[index] = states[index].received_at;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DamiaoSystemHardware::write(const rclcpp::Time&,
    const rclcpp::Duration&)
{
    if (fault_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"), "Write rejected: hardware fault is latched.");
        return hardware_interface::return_type::ERROR;
    }
    if (!active_ || !bus_)
    {
        return hardware_interface::return_type::OK;
    }
    // 激活已发出保持目标；按实际发送间隔检查后续周期和关节速度。
    const auto send_at = damiao::SteadyClock::now();
    const double seconds = std::chrono::duration<double>(send_at - last_command_sent_at_).count();
    if (last_command_sent_at_ == damiao::Deadline{} || !std::isfinite(seconds)
        || seconds < 0.0 || seconds > std::chrono::duration<double>(config_.max_control_period).count())
    {
        RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
            "Write failed: command interval %.3f ms exceeds %.3f ms or has no valid timestamp",
            seconds * 1000.0, std::chrono::duration<double, std::milli>(config_.max_control_period).count());
        fault_ = true;
        bus_->end_control();
        return hardware_interface::return_type::ERROR;
    }
    // 初始保持目标已发出；等待至少一个 100 Hz 周期，避免两组六帧挤满 can0 发送队列。
    if (initial_hold_pending_ && send_at - last_command_sent_at_ < std::chrono::milliseconds(10))
    {
        return hardware_interface::return_type::OK;
    }
    initial_hold_pending_ = false;
    std::array<damiao::PositionVelocityCommand, damiao::max_motors> targets{};
    std::array<damiao::ErrorCode, damiao::max_motors> results{};
    std::array<double, damiao::max_motors> bounded_commands{};
    std::size_t first_limited_index = axis_count_;
    // 整组目标先检查有限性和限位；速度跳变只限制本周期实际发送值。
    for (std::size_t index = 0; index < axis_count_; ++index)
    {
        const auto& axis = axes_[index];
        const double command = position_command_[index];
        if (!std::isfinite(command) || command < axis.min_position_rad
            || command > axis.max_position_rad)
        {
            RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
                "Write failed for %s: target=%.6f limits=[%.6f, %.6f]",
                axis.joint_name.c_str(), command, axis.min_position_rad, axis.max_position_rad);
            fault_ = true;
            bus_->end_control();
            return hardware_interface::return_type::ERROR;
        }
        bounded_commands[index] = limit_position_step(command, last_command_[index],
            axis.max_velocity_rad_s, seconds);
        if (bounded_commands[index] != command && first_limited_index == axis_count_)
        {
            first_limited_index = index;
        }
        targets[index] = {motor_position(index, bounded_commands[index]),
            axis.extra_reduction * axis.max_velocity_rad_s};
    }
    const auto sent = bus_->send_position_velocity_batch(targets.data(), axis_count_, results.data());
    if (sent != damiao::ErrorCode::Ok)
    {
        // 找出第一台未成功发送的电机，方便区分反馈故障与部分发送故障。
        std::size_t failed_index = 0;
        while (failed_index < axis_count_ && results[failed_index] == damiao::ErrorCode::Ok)
        {
            ++failed_index;
        }
        RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
            "Write failed: CAN batch error=%d first_failed_axis=%zu axis_error=%d",
            static_cast<int>(sent), failed_index < axis_count_ ? failed_index + 1 : 0,
            failed_index < axis_count_ ? static_cast<int>(results[failed_index]) : -1);
        fault_ = true;
        bus_->end_control();
        return hardware_interface::return_type::ERROR;
    }
    // 整批成功后才报告实际限幅，避免把未发送的目标记为已发送。
    if (first_limited_index < axis_count_
        && (last_rate_warning_at_ == damiao::Deadline{}
            || send_at - last_rate_warning_at_ >= std::chrono::seconds(1)))
    {
        RCLCPP_WARN(rclcpp::get_logger("damiao_hardware"),
            "Rate-limited %s: requested=%.6f sent=%.6f previous=%.6f interval=%.3f ms",
            axes_[first_limited_index].joint_name.c_str(),
            position_command_[first_limited_index], bounded_commands[first_limited_index],
            last_command_[first_limited_index], seconds * 1000.0);
        last_rate_warning_at_ = send_at;
    }
    last_command_sent_at_ = send_at;
    for (std::size_t index = 0; index < axis_count_; ++index)
    {
        last_command_[index] = bounded_commands[index];
    }
    return hardware_interface::return_type::OK;
}

void DamiaoSystemHardware::start_inactive_polling()
{
    stop_poller_ = false;
    inactive_poller_ = std::thread([this]()
    {
        while (!stop_poller_)
        {
            // 失能期顺序查询六轴；核心缓存保留各轴原始接收时间。
            for (std::size_t index = 0; index < axis_count_ && !stop_poller_; ++index)
            {
                bus_->query_state(index, damiao::SteadyClock::now() + config_.management_timeout);
            }
            const auto until = damiao::SteadyClock::now() + config_.inactive_poll_period;
            while (!stop_poller_ && damiao::SteadyClock::now() < until)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    });
}

void DamiaoSystemHardware::stop_inactive_polling()
{
    stop_poller_ = true;
    if (inactive_poller_.joinable())
    {
        inactive_poller_.join();
    }
}

bool DamiaoSystemHardware::disable_all()
{
    if (!bus_)
    {
        return true;
    }
    bus_->end_control();
    bool all_disabled = true;
    for (std::size_t index = 0; index < axis_count_; ++index)
    {
        const auto disabled = bus_->disable(
            index, damiao::SteadyClock::now() + config_.management_timeout);
        if (disabled.code != damiao::ErrorCode::Ok)
        {
            RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
                "Cannot confirm disable for %s: %s",
                axes_[index].motor_name.c_str(), disabled.message.c_str());
            all_disabled = false;
        }
    }
    if (all_disabled)
    {
        enable_attempted_ = false;
        active_ = false;
    }
    return all_disabled;
}

void DamiaoSystemHardware::close_bus()
{
    stop_inactive_polling();
    if (bus_)
    {
        if (enable_attempted_)
        {
            disable_all();
        }
        const auto closed = bus_->close();
        if (closed.code != damiao::ErrorCode::Ok)
        {
            RCLCPP_ERROR(rclcpp::get_logger("damiao_hardware"),
                "Bus close failed: %s", closed.message.c_str());
        }
        bus_.reset();
    }
    configured_ = false;
    active_ = false;
    enable_attempted_ = false;
    last_command_sent_at_ = {};
    last_rate_warning_at_ = {};
    initial_hold_pending_ = false;
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    position_command_.fill(invalid);
    last_command_.fill(invalid);
}

bool DamiaoSystemHardware::decode_state(std::size_t index, const damiao::MotorState& state,
    double& position, double& velocity) const
{
    const auto& axis = axes_[index];
    if (!state.valid || !std::isfinite(state.output_position_rad)
        || !std::isfinite(state.output_velocity_rad_s))
    {
        return false;
    }
    position = axis.direction * (state.output_position_rad - axis.zero_offset_motor_output_rad)
        / axis.extra_reduction;
    velocity = axis.direction * state.output_velocity_rad_s / axis.extra_reduction;
    return std::isfinite(position) && std::isfinite(velocity);
}

bool DamiaoSystemHardware::position_within_limits(std::size_t index, double position) const
{
    const auto& axis = axes_[index];
    return position >= axis.min_position_rad && position <= axis.max_position_rad;
}

double DamiaoSystemHardware::motor_position(std::size_t index, double joint_position) const
{
    const auto& axis = axes_[index];
    return axis.zero_offset_motor_output_rad
        + axis.direction * axis.extra_reduction * joint_position;
}

}  // namespace damiao_hardware

PLUGINLIB_EXPORT_CLASS(damiao_hardware::DamiaoSystemHardware, hardware_interface::SystemInterface)
