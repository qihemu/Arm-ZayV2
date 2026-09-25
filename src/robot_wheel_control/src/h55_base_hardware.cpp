#include <robot_wheel_control/h55_base_hardware.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace robot_wheel_control
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
H55BaseHardware::~H55BaseHardware()
{
    close();
}
CallbackReturn H55BaseHardware::on_init(const hardware_interface::HardwareInfo &info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS || info.joints.size() != 2)
    {
        return CallbackReturn::ERROR;
    }
    for (const auto &j : info.joints)
    {
        if (j.command_interfaces.size() != 1 || j.command_interfaces[0].name != "velocity" ||
            j.state_interfaces.size() != 2 || j.state_interfaces[0].name != "position" ||
            j.state_interfaces[1].name != "velocity")
        {
            return CallbackReturn::ERROR;
        }
    }
    return CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface> H55BaseHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> result;
    for (std::size_t i = 0; i < 2; ++i)
    {
        result.emplace_back(info_.joints[i].name, "position", &position_[i]);
        result.emplace_back(info_.joints[i].name, "velocity", &velocity_[i]);
    }
    return result;
}
std::vector<hardware_interface::CommandInterface> H55BaseHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> result;
    result.emplace_back(info_.joints[0].name, "velocity", &command_[0]);
    result.emplace_back(info_.joints[1].name, "velocity", &command_[1]);
    return result;
}
CallbackReturn H55BaseHardware::on_configure(const rclcpp_lifecycle::State &)
{
    try
    {
        auto c = load_configuration(info_.hardware_parameters.at("config_file"),
                                    info_.hardware_parameters.at("backend"), "base");
        for (std::size_t i = 0; i < 2; ++i)
        {
            if (info_.joints[i].name != c.joint_names[i])
            {
                throw std::runtime_error("URDF/YAML wheel mismatch");
            }
        }
        runtime_ = std::make_shared<WheelRuntime>(c);
        api_ = std::make_shared<WheelRosApi>(runtime_);
        runtime_->start();
        executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(),
                                                                               c.executor_threads);
        executor_->add_node(api_);
        api_thread_ = std::thread([this] { executor_->spin(); });
        return CallbackReturn::SUCCESS;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("H55BaseHardware"), "%s", e.what());
        close();
        return CallbackReturn::ERROR;
    }
}
CallbackReturn H55BaseHardware::on_activate(const rclcpp_lifecycle::State &)
{
    // 激活ROS接口不使能电机，显式set_enabled服务才授予运动许可。
    command_ = {};
    return runtime_ ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}
CallbackReturn H55BaseHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
    command_ = {};
    if (runtime_)
    {
        std::string reason;
        runtime_->submit(Operation::Stop, "",
                         "deactivate-" +
                             std::to_string(damiao::SteadyClock::now().time_since_epoch().count()),
                         0, true, reason);
    }
    return CallbackReturn::SUCCESS;
}
void H55BaseHardware::close()
{
    if (executor_)
    {
        executor_->cancel();
    }
    if (api_thread_.joinable())
    {
        api_thread_.join();
    }
    if (runtime_)
    {
        runtime_->shutdown();
    }
    executor_.reset();
    api_.reset();
    runtime_.reset();
}
CallbackReturn H55BaseHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
    close();
    return CallbackReturn::SUCCESS;
}
CallbackReturn H55BaseHardware::on_shutdown(const rclcpp_lifecycle::State &)
{
    close();
    return CallbackReturn::SUCCESS;
}
CallbackReturn H55BaseHardware::on_error(const rclcpp_lifecycle::State &)
{
    close();
    return CallbackReturn::SUCCESS;
}
return_type H55BaseHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (!runtime_)
    {
        return return_type::ERROR;
    }
    const auto s = runtime_->snapshot();
    if (s.position_valid)
    {
        position_ = s.position;
        velocity_ = s.velocity;
    }
    // 启动初值只用于禁用态控制器初始化；真实里程许可由/base/state明确给出。
    // Runtime已锁存故障并停车；保持API存活以报告原因。wheel_odom有效性看/base/state。
    return return_type::OK;
}
return_type H55BaseHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (!runtime_)
    {
        return return_type::ERROR;
    }
    const auto s = runtime_->snapshot();
    if (s.enabled && s.permitted && !runtime_->command(command_, true))
    {
        runtime_->request_fault("Invalid controller wheel command");
        return return_type::OK;
    }
    return return_type::OK;
}
} // namespace robot_wheel_control
PLUGINLIB_EXPORT_CLASS(robot_wheel_control::H55BaseHardware, hardware_interface::SystemInterface)
