#pragma once
#include <robot_wheel_control/runtime.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <robot_interfaces/msg/wheel_base_state.hpp>
#include <robot_interfaces/msg/wheel_velocity_command.hpp>
#include <robot_interfaces/msg/wheel_control_event.hpp>
#include <robot_interfaces/srv/set_wheel_base_enabled.hpp>
#include <robot_interfaces/srv/stop_wheel_base.hpp>
#include <robot_interfaces/srv/clear_wheel_base_fault.hpp>
#include <robot_interfaces/srv/get_wheel_base_state.hpp>
#include <robot_interfaces/srv/get_wheel_control_result.hpp>
#include <robot_interfaces/srv/move_wheel_base_relative.hpp>
#include <robot_interfaces/msg/wheel_motion_heartbeat.hpp>
namespace robot_wheel_control
{
// ROS层仅操作本实例的缓存/队列；服务不直接执行CAN事务。
class WheelRosApi : public rclcpp::Node
{
  public:
    explicit WheelRosApi(std::shared_ptr<WheelRuntime> runtime);

  private:
    bool fresh(const builtin_interfaces::msg::Time &stamp);
    robot_interfaces::msg::WheelBaseState message();
    void publish();
    std::shared_ptr<WheelRuntime> runtime_;
    rclcpp::CallbackGroup::SharedPtr commands_, management_, status_;
    rclcpp::Publisher<robot_interfaces::msg::WheelBaseState>::SharedPtr states_;
    rclcpp::Publisher<robot_interfaces::msg::WheelControlEvent>::SharedPtr events_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_out_;
    rclcpp::Subscription<robot_interfaces::msg::WheelVelocityCommand>::SharedPtr wheels_in_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_in_;
    rclcpp::Service<robot_interfaces::srv::SetWheelBaseEnabled>::SharedPtr enable_;
    rclcpp::Service<robot_interfaces::srv::StopWheelBase>::SharedPtr stop_;
    rclcpp::Service<robot_interfaces::srv::ClearWheelBaseFault>::SharedPtr clear_;
    rclcpp::Service<robot_interfaces::srv::GetWheelBaseState>::SharedPtr get_;
    rclcpp::Service<robot_interfaces::srv::GetWheelControlResult>::SharedPtr result_;
    rclcpp::Service<robot_interfaces::srv::MoveWheelBaseRelative>::SharedPtr relative_;
    rclcpp::Subscription<robot_interfaces::msg::WheelMotionHeartbeat>::SharedPtr heartbeat_;
    rclcpp::Time last_heartbeat_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::TimerBase::SharedPtr timer_;
    std::array<std::uint64_t, 2> stamp_sequence_{};
    std::array<builtin_interfaces::msg::Time, 2> sample_stamp_{builtin_interfaces::msg::Time(),
                                                               builtin_interfaces::msg::Time()};
    rclcpp::Time last_command_stamp_{0, 0, RCL_ROS_TIME};
};
} // namespace robot_wheel_control
