#include <damiao_hardware/damiao_system_hardware.hpp>
#include <damiao_hardware/position_step_limit.hpp>

#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>
#include <pluginlib/class_loader.hpp>

#include <cmath>
#include <sstream>

namespace
{

// 用完整的单轴描述测试配置和稳定接口，不打开 CAN 设备。
hardware_interface::HardwareInfo valid_info()
{
    hardware_interface::HardwareInfo info;
    info.name = "DamiaoArm";
    info.type = "system";
    info.hardware_parameters = {
        {"motor_name", "motor_1"}, {"model", "J4340-2EC"}, {"can_interface", "vcan0"},
        {"esc_id", "1"}, {"mst_id", "17"}, {"feedback_timeout_ms", "1000"},
        {"management_timeout_ms", "700"}, {"management_quiet_period_ms", "2"},
        {"inactive_poll_period_ms", "100"}, {"max_control_period_ms", "50"}
    };
    hardware_interface::ComponentInfo joint;
    joint.name = "joint1";
    joint.command_interfaces = {{"position"}};
    joint.state_interfaces = {{"position"}, {"velocity"}};
    joint.parameters = {
        {"motor_name", "motor_1"}, {"direction", "1"},
        {"zero_offset_motor_output_rad", "0.5"}, {"extra_reduction", "1.0"},
        {"min_position_rad", "-1.0"}, {"max_position_rad", "1.0"},
        {"max_velocity_rad_s", "0.5"}, {"activation_position_tolerance_rad", "0.05"}
    };
    info.joints.push_back(joint);
    return info;
}

// 六轴夹具使用唯一地址和固定顺序，验证插件整组接口的映射。
hardware_interface::HardwareInfo valid_six_axis_info()
{
    auto info = valid_info();
    info.hardware_parameters.erase("motor_name");
    info.hardware_parameters.erase("model");
    info.hardware_parameters.erase("esc_id");
    info.hardware_parameters.erase("mst_id");
    info.hardware_parameters["management_timeout_ms"] = "100";
    info.joints.clear();
    for (int number = 1; number <= 6; ++number)
    {
        auto joint = valid_info().joints[0];
        joint.name = "joint" + std::to_string(number);
        joint.parameters["motor_name"] = "motor_" + std::to_string(number);
        joint.parameters["model"] = "J4340-2EC";
        joint.parameters["esc_id"] = std::to_string(number);
        joint.parameters["mst_id"] = std::to_string(16 + number);
        info.joints.push_back(joint);
    }
    return info;
}

// 新轨迹替换时只允许按已发送目标和实际周期渐进靠近，反向变化同样受限。
TEST(DamiaoHardware, LimitsReplannedCommandByActualSendInterval)
{
    constexpr double previous = -0.188262;
    constexpr double requested = -0.127413;
    constexpr double interval = 0.009968;
    const double bounded = damiao_hardware::limit_position_step(requested, previous, 3.0, interval);
    EXPECT_NEAR(bounded, previous + 3.0 * interval * 0.9, 1e-12);
    EXPECT_LT(bounded, requested);
    EXPECT_DOUBLE_EQ(damiao_hardware::limit_position_step(previous - 0.001, previous,
        3.0, interval), previous - 0.001);
    EXPECT_NEAR(damiao_hardware::limit_position_step(previous - 0.1, previous,
        3.0, interval), previous - 3.0 * interval * 0.9, 1e-12);
}

TEST(DamiaoHardware, ExportsSingleAxisWithoutMovement)
{
    damiao_hardware::DamiaoSystemHardware hardware;
    ASSERT_EQ(hardware.on_init(valid_info()), hardware_interface::CallbackReturn::SUCCESS);
    auto states = hardware.export_state_interfaces();
    auto commands = hardware.export_command_interfaces();
    ASSERT_EQ(states.size(), 2U);
    ASSERT_EQ(commands.size(), 1U);
    EXPECT_EQ(states[0].get_name(), "joint1/position");
    EXPECT_EQ(states[1].get_name(), "joint1/velocity");
    EXPECT_EQ(commands[0].get_name(), "joint1/position");
    EXPECT_TRUE(std::isnan(states[0].get_value()));
    EXPECT_TRUE(std::isnan(commands[0].get_value()));
    EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
}

TEST(DamiaoHardware, RejectsInvalidSingleAxisConfiguration)
{
    auto info = valid_info();
    info.joints[0].parameters["motor_name"] = "wrong_motor";
    damiao_hardware::DamiaoSystemHardware mismatch;
    EXPECT_EQ(mismatch.on_init(info), hardware_interface::CallbackReturn::ERROR);
    info = valid_info();
    info.joints[0].parameters["max_velocity_rad_s"] = "nan";
    damiao_hardware::DamiaoSystemHardware nonfinite;
    EXPECT_EQ(nonfinite.on_init(info), hardware_interface::CallbackReturn::ERROR);
    info = valid_info();
    info.joints[0].command_interfaces.push_back({"velocity"});
    damiao_hardware::DamiaoSystemHardware extra_interface;
    EXPECT_EQ(extra_interface.on_init(info), hardware_interface::CallbackReturn::ERROR);
}

TEST(DamiaoHardware, ExportsSixAxesInBusOrderWithoutMovement)
{
    damiao_hardware::DamiaoSystemHardware hardware;
    ASSERT_EQ(hardware.on_init(valid_six_axis_info()), hardware_interface::CallbackReturn::SUCCESS);
    auto states = hardware.export_state_interfaces();
    auto commands = hardware.export_command_interfaces();
    ASSERT_EQ(states.size(), 12U);
    ASSERT_EQ(commands.size(), 6U);
    for (int number = 1; number <= 6; ++number)
    {
        const auto index = static_cast<std::size_t>(number - 1);
        const auto name = "joint" + std::to_string(number);
        EXPECT_EQ(states[2 * index].get_name(), name + "/position");
        EXPECT_EQ(states[2 * index + 1].get_name(), name + "/velocity");
        EXPECT_EQ(commands[index].get_name(), name + "/position");
        EXPECT_TRUE(std::isnan(states[2 * index].get_value()));
        EXPECT_TRUE(std::isnan(commands[index].get_value()));
    }
    EXPECT_EQ(hardware.write(rclcpp::Time(), rclcpp::Duration(0, 10000000)),
        hardware_interface::return_type::OK);
}

TEST(DamiaoHardware, RejectsSixAxisMappingErrors)
{
    auto info = valid_six_axis_info();
    info.joints[5].parameters["esc_id"] = "1";
    damiao_hardware::DamiaoSystemHardware duplicate;
    EXPECT_EQ(duplicate.on_init(info), hardware_interface::CallbackReturn::ERROR);
    info = valid_six_axis_info();
    info.joints[5].parameters["direction"] = "0";
    damiao_hardware::DamiaoSystemHardware direction;
    EXPECT_EQ(direction.on_init(info), hardware_interface::CallbackReturn::ERROR);
    info = valid_six_axis_info();
    info.hardware_parameters["feedback_timeout_ms"] = "200";
    damiao_hardware::DamiaoSystemHardware deadline;
    EXPECT_EQ(deadline.on_init(info), hardware_interface::CallbackReturn::ERROR);
}

TEST(DamiaoHardware, ResourceManagerLoadsSixAxesWithoutCan)
{
    std::ostringstream urdf;
    urdf << "<robot name=\"six_axis\"><ros2_control name=\"DamiaoArm\" type=\"system\">";
    urdf << "<hardware><plugin>damiao_hardware/DamiaoSystemHardware</plugin>";
    for (const auto& item : valid_six_axis_info().hardware_parameters)
    {
        urdf << "<param name=\"" << item.first << "\">" << item.second << "</param>";
    }
    urdf << "</hardware>";
    for (const auto& joint : valid_six_axis_info().joints)
    {
        urdf << "<joint name=\"" << joint.name << "\">";
        for (const auto& item : joint.parameters)
        {
            urdf << "<param name=\"" << item.first << "\">" << item.second << "</param>";
        }
        urdf << "<command_interface name=\"position\"/>";
        urdf << "<state_interface name=\"position\"/>";
        urdf << "<state_interface name=\"velocity\"/></joint>";
    }
    urdf << "</ros2_control></robot>";
    hardware_interface::ResourceManager resources(urdf.str(), true, false);
    for (int number = 1; number <= 6; ++number)
    {
        const auto name = "joint" + std::to_string(number);
        EXPECT_TRUE(resources.command_interface_exists(name + "/position"));
        EXPECT_TRUE(resources.state_interface_exists(name + "/position"));
        EXPECT_TRUE(resources.state_interface_exists(name + "/velocity"));
    }
}

TEST(DamiaoHardware, PluginlibDiscoversInstalledClass)
{
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
        "hardware_interface", "hardware_interface::SystemInterface");
    EXPECT_TRUE(loader.isClassAvailable("damiao_hardware/DamiaoSystemHardware"));
    auto hardware = loader.createSharedInstance("damiao_hardware/DamiaoSystemHardware");
    ASSERT_NE(hardware, nullptr);
    EXPECT_EQ(hardware->on_init(valid_info()), hardware_interface::CallbackReturn::SUCCESS);
}


TEST(DamiaoHardware, ResourceManagerLoadsWithoutOpeningCan)
{
    // 资源管理器只执行 on_init；不存在的 vcan 名称证明加载阶段不会访问总线。
    const std::string urdf = R"(<robot name="single_axis">
        <ros2_control name="DamiaoArm" type="system">
            <hardware>
                <plugin>damiao_hardware/DamiaoSystemHardware</plugin>
                <param name="motor_name">motor_1</param>
                <param name="model">J4340-2EC</param>
                <param name="can_interface">vcan_dev07_missing</param>
                <param name="esc_id">1</param>
                <param name="mst_id">17</param>
                <param name="feedback_timeout_ms">1000</param>
                <param name="management_timeout_ms">700</param>
                <param name="management_quiet_period_ms">2</param>
                <param name="inactive_poll_period_ms">100</param>
                <param name="max_control_period_ms">50</param>
            </hardware>
            <joint name="joint1">
                <param name="motor_name">motor_1</param>
                <param name="direction">1</param>
                <param name="zero_offset_motor_output_rad">0.5</param>
                <param name="extra_reduction">1.0</param>
                <param name="min_position_rad">-1.0</param>
                <param name="max_position_rad">1.0</param>
                <param name="max_velocity_rad_s">0.5</param>
                <param name="activation_position_tolerance_rad">0.05</param>
                <command_interface name="position"/>
                <state_interface name="position"/>
                <state_interface name="velocity"/>
            </joint>
        </ros2_control>
    </robot>)";
    hardware_interface::ResourceManager resources(urdf, true, false);
    EXPECT_TRUE(resources.command_interface_exists("joint1/position"));
    EXPECT_TRUE(resources.state_interface_exists("joint1/position"));
    EXPECT_TRUE(resources.state_interface_exists("joint1/velocity"));
}

}  // namespace
