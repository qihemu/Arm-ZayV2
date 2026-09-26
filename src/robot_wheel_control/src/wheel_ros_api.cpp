#include <robot_wheel_control/ros_api.hpp>
#include <cmath>
#include <limits>
namespace robot_wheel_control
{
using namespace robot_interfaces;
WheelRosApi::WheelRosApi(std::shared_ptr<WheelRuntime> r)
    : Node("wheel_api", "/base", rclcpp::NodeOptions().use_global_arguments(false)), runtime_(std::move(r))
{
    commands_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    management_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    status_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    states_ = create_publisher<msg::WheelBaseState>("state", rclcpp::QoS(1).reliable());
    events_ = create_publisher<msg::WheelControlEvent>("control_events", rclcpp::QoS(32).reliable());
    if (runtime_->configuration().mode != "base")
    {
        joints_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    }
    twist_out_ =
        create_publisher<geometry_msgs::msg::TwistStamped>("diff_drive_controller/cmd_vel", rclcpp::QoS(1));
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = commands_;
    wheels_in_ = create_subscription<msg::WheelVelocityCommand>(
        "wheel_velocity", rclcpp::QoS(1),
        [this](msg::WheelVelocityCommand::SharedPtr m)
        {
            const auto s = runtime_->snapshot();
            if (runtime_->configuration().mode == "base" || m->session_id != s.session ||
                !fresh(m->header.stamp))
            {
                return;
            }
            if (s.permitted && !s.relative_active && !runtime_->command({m->left_rad_s, m->right_rad_s}, false, m->source_id))
            {
                runtime_->request_fault("Invalid wheel command");
            }
        },
        sub_options);
    twist_in_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", rclcpp::QoS(1),
        [this](geometry_msgs::msg::TwistStamped::SharedPtr m)
        {
            if (runtime_->configuration().mode != "base" || !fresh(m->header.stamp))
            {
                return;
            }
            const auto &t = m->twist;
            if (!std::isfinite(t.linear.x) || !std::isfinite(t.angular.z) || t.linear.y != 0 ||
                t.linear.z != 0 || t.angular.x != 0 || t.angular.y != 0)
            {
                runtime_->request_fault("Invalid differential-drive Twist");
                return;
            }
            const auto s = runtime_->snapshot();
            if (!s.enabled || !s.permitted || s.fault)
            {
                return;
            }
            runtime_->authorize_source();
            twist_out_->publish(*m);
        },
        sub_options);
    // Serve immutable effective YAML; clients never infer limits from a local cache.
    configuration_ = create_service<srv::GetWheelConfiguration>("get_configuration",
        [this](const std::shared_ptr<srv::GetWheelConfiguration::Request>,
               std::shared_ptr<srv::GetWheelConfiguration::Response> p)
        {
            const auto &c = runtime_->configuration();
            p->session_id = runtime_->snapshot().session;
            p->configuration_digest = c.digest;
            p->source_path = c.source_path;
            p->configuration_yaml = c.yaml;
        }, rmw_qos_profile_services_default, status_);
    enable_ = create_service<srv::SetWheelBaseEnabled>(
        "set_enabled",
        [this](const std::shared_ptr<srv::SetWheelBaseEnabled::Request> q,
               std::shared_ptr<srv::SetWheelBaseEnabled::Response> p)
        {
            p->accepted = runtime_->submit(q->enable ? Operation::Enable : Operation::Disable, q->session_id,
                                           q->request_id, 0, true, p->reason);
            p->session_id = runtime_->snapshot().session;
            p->request_id = q->request_id;
        },
        rmw_qos_profile_services_default, management_);
    relative_ = create_service<srv::MoveWheelBaseRelative>(
        "move_relative",
        [this](const std::shared_ptr<srv::MoveWheelBaseRelative::Request> q,
               std::shared_ptr<srv::MoveWheelBaseRelative::Response> p)
        {
            p->session_id = runtime_->snapshot().session;
            p->request_id = q->request_id;
            if (!q->enable)
            {
                p->reason = "Relative task requires explicit enable=true";
                return;
            }
            p->accepted =
                runtime_->submit(Operation::Relative, q->session_id, q->request_id, 0, true, p->reason,
                                 RelativeGoal{q->kind, q->value, q->max_speed, q->timeout_s});
        },
        rmw_qos_profile_services_default, management_);
    heartbeat_ = create_subscription<msg::WheelMotionHeartbeat>(
        "relative_keepalive", rclcpp::QoS(1),
        [this](msg::WheelMotionHeartbeat::SharedPtr m)
        {
            // 心跳单独去重，不能受无关手动速度消息的时间戳干扰。
            const auto stamp = rclcpp::Time(m->header.stamp, get_clock()->get_clock_type());
            const double age = (get_clock()->now() - stamp).seconds();
            if (stamp.nanoseconds() > 0 && stamp > last_heartbeat_stamp_ && age >= 0 &&
                age <= runtime_->configuration().relative.heartbeat_ms / 1000.0 &&
                runtime_->relative_heartbeat(m->session_id, m->request_id))
            {
                last_heartbeat_stamp_ = stamp;
            }
        },
        sub_options);
    stop_ = create_service<srv::StopWheelBase>(
        "stop",
        [this](const std::shared_ptr<srv::StopWheelBase::Request> q,
               std::shared_ptr<srv::StopWheelBase::Response> p)
        {
            p->accepted =
                runtime_->submit(Operation::Stop, "", q->request_id, 0, q->disable_after_stop, p->reason);
            p->session_id = runtime_->snapshot().session;
            p->request_id = q->request_id;
        },
        rmw_qos_profile_services_default, management_);
    clear_ = create_service<srv::ClearWheelBaseFault>(
        "clear_fault",
        [this](const std::shared_ptr<srv::ClearWheelBaseFault::Request> q,
               std::shared_ptr<srv::ClearWheelBaseFault::Response> p)
        {
            p->accepted = runtime_->submit(Operation::Clear, q->session_id, q->request_id,
                                           q->expected_fault_sequence, true, p->reason);
            p->session_id = runtime_->snapshot().session;
            p->request_id = q->request_id;
        },
        rmw_qos_profile_services_default, management_);
    get_ = create_service<srv::GetWheelBaseState>(
        "get_state",
        [this](const std::shared_ptr<srv::GetWheelBaseState::Request>,
               std::shared_ptr<srv::GetWheelBaseState::Response> p)
        {
            p->available = true;
            p->state = message();
        },
        rmw_qos_profile_services_default, status_);
    result_ = create_service<srv::GetWheelControlResult>(
        "get_control_result",
        [this](const std::shared_ptr<srv::GetWheelControlResult::Request> q,
               std::shared_ptr<srv::GetWheelControlResult::Response> p)
        {
            p->session_id = runtime_->snapshot().session;
            p->request_id = q->request_id;
            if (q->session_id != p->session_id)
            {
                p->status = 0;
                p->reason = "Session mismatch";
                return;
            }
            const auto r = runtime_->result(q->request_id);
            p->status = r.status;
            p->reason = r.reason;
        },
        rmw_qos_profile_services_default, status_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1 / runtime_->configuration().status_hz)),
        [this] { publish(); }, status_);
}
bool WheelRosApi::fresh(const builtin_interfaces::msg::Time &stamp)
{
    const auto time = rclcpp::Time(stamp, get_clock()->get_clock_type());
    const auto now = get_clock()->now();
    const double age = (now - time).seconds();
    if (time.nanoseconds() == 0 || age < 0 || age > runtime_->configuration().command_timeout_ms / 1000.0 ||
        time <= last_command_stamp_)
    {
        return false;
    }
    last_command_stamp_ = time;
    return true;
}
msg::WheelBaseState WheelRosApi::message()
{
    const auto s = runtime_->snapshot();
    const auto &c = runtime_->configuration();
    const auto steady = damiao::SteadyClock::now();
    const auto now = get_clock()->now();
    msg::WheelBaseState out;
    out.header.stamp = now;
    out.header.frame_id = "base_link";
    out.backend = c.backend;
    out.operation_mode = c.mode;
    out.session_id = s.session;
    out.configuration_digest = c.digest;
    out.state_sequence = s.sequence;
    out.fault_sequence = s.fault_sequence;
    out.lifecycle_state = s.lifecycle;
    out.motion_authorized = s.permitted;
    out.command_fresh = s.command_fresh;
    out.odometry_valid = c.mode == "base" && s.position_valid && !s.fault;
    out.standstill_confirmed = s.standstill && !s.fault;
    // 驻车硬件尚未接入，不能由零速或失能推断双臂作业许可。
    out.parking_confirmed = false;
    out.arm_operation_permitted = false;
    out.emergency_stop_state = msg::WheelBaseState::ESTOP_UNKNOWN;
    out.last_request_id = s.last_request;
    out.last_request_state = s.last_result;
    out.reason = s.reason;
    out.action_wheel_travel_m = s.action_travel;
    out.action_distance_limit_m = c.action_distance;
    out.relative_timeout_s = s.relative_goal.timeout_s;
    out.relative_motion_active = s.relative_active;
    out.relative_request_id = s.relative_id;
    out.relative_kind = s.relative_goal.kind;
    out.relative_target = s.relative_goal.value;
    out.relative_measured = s.relative_measured;
    out.relative_error = s.relative_goal.value - s.relative_measured;
    out.relative_wheel_target_rad = s.relative_target;
    out.relative_wheel_travel_rad = s.relative_travel;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < 2; ++i)
    {
        const auto &raw = s.motors[i];
        auto &m = i == 0 ? out.left : out.right;
        const double age =
            raw.valid ? std::max(0.0, std::chrono::duration<double>(steady - raw.received_at).count())
                      : 2147483646.0;
        if (raw.valid && raw.sequence != stamp_sequence_[i])
        {
            sample_stamp_[i] = now - rclcpp::Duration::from_seconds(age);
            stamp_sequence_[i] = raw.sequence;
        }
        m.header.stamp = sample_stamp_[i];
        m.header.frame_id = "base_link";
        m.sample_sequence = raw.sequence;
        m.joint_name = c.joint_names[i];
        m.can_id = c.bus.motors[i].address.esc_id;
        m.master_id = c.bus.motors[i].address.mst_id;
        m.control_mode = 3;
        m.direction = c.direction[i];
        m.feedback_age = rclcpp::Duration::from_seconds(age);
        const double feedback_timeout =
            s.enabled ? c.bus.feedback_timeout.count() / 1000.0 : c.idle_feedback_timeout_ms / 1000.0;
        m.feedback_valid = raw.valid && age <= feedback_timeout;
        m.identity_verified = s.configured;
        m.continuous_position_valid = s.position_valid && m.feedback_valid;
        m.driver_enabled = m.feedback_valid && raw.raw_status == 1;
        m.raw_state_code = raw.raw_status;
        m.raw_position_rad = raw.valid ? raw.output_position_rad : nan;
        m.raw_velocity_rad_s = raw.valid ? raw.output_velocity_rad_s : nan;
        m.continuous_position_rad = m.continuous_position_valid ? s.position[i] : nan;
        m.velocity_rad_s = m.feedback_valid ? s.velocity[i] : nan;
        m.target_velocity_rad_s = s.target[i];
        m.reported_torque_nm = raw.valid ? raw.reported_torque_nm : nan;
        m.driver_temperature_c = raw.valid ? raw.mos_temperature_c : nan;
        m.motor_temperature_c = raw.valid ? raw.rotor_temperature_c : nan;
    }
    out.left_travel_m = out.odometry_valid ? s.position[0] * c.radius[0] : nan;
    out.right_travel_m = out.odometry_valid ? s.position[1] * c.radius[1] : nan;
    out.base_path_length_m = out.odometry_valid ? s.path : nan;
    return out;
}
void WheelRosApi::publish()
{
    auto state = message();
    states_->publish(state);
    if (joints_)
    {
        sensor_msgs::msg::JointState joint;
        joint.header = state.header;
        joint.name = {state.left.joint_name, state.right.joint_name};
        joint.position = {state.left.continuous_position_rad, state.right.continuous_position_rad};
        joint.velocity = {state.left.velocity_rad_s, state.right.velocity_rad_s};
        joints_->publish(joint);
    }
    for (const auto &r : runtime_->take_events())
    {
        msg::WheelControlEvent e;
        e.header.stamp = get_clock()->now();
        e.session_id = state.session_id;
        e.request_id = r.id;
        e.result = r.status == 2 ? 1 : (r.status == 3 ? 2 : 3);
        e.state_sequence = state.state_sequence;
        e.reason = r.reason;
        events_->publish(e);
    }
}
} // namespace robot_wheel_control
