#pragma once
#include <robot_wheel_control/ros_api.hpp>
#include <hardware_interface/system_interface.hpp>
namespace robot_wheel_control
{
// ROS硬件插件仅导出两个continuous轮关节，命令为velocity。
class H55BaseHardware final : public hardware_interface::SystemInterface
{
  public:
    ~H55BaseHardware() override;
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  private:
    void close();
    std::shared_ptr<WheelRuntime> runtime_;
    std::shared_ptr<WheelRosApi> api_;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread api_thread_;
    std::array<double, 2> position_{}, velocity_{}, command_{};
};
} // namespace robot_wheel_control
