#include <functional>
#include <memory>

#include "arm_control/arm_driver_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace arm_control
{

ArmDriverNode::ArmDriverNode()
    : node_(rclcpp::Node::make_shared("arm_driver_node")),
      controller_(node_)
{
    controller_.initialize();
    setupServices();
}

void ArmDriverNode::setupServices()
{
    move_to_point_service_ = node_->create_service<robot_interfaces::srv::ArmMoveToPoint>(
        "/arm/move_to_point",
        std::bind(&ArmDriverNode::handleMoveToPoint, this, std::placeholders::_1, std::placeholders::_2));

    set_preset_pose_service_ = node_->create_service<robot_interfaces::srv::SetPresetPose>(
        "/arm/set_preset_pose",
        std::bind(&ArmDriverNode::handleSetPresetPose, this, std::placeholders::_1, std::placeholders::_2));
}

void ArmDriverNode::handleMoveToPoint(
    const std::shared_ptr<robot_interfaces::srv::ArmMoveToPoint::Request> request,
    std::shared_ptr<robot_interfaces::srv::ArmMoveToPoint::Response> response)
{
    RCLCPP_INFO(node_->get_logger(), "Received move_to_point request");
    response->success = controller_.moveToWayPoint(request->target_point);
}

void ArmDriverNode::handleSetPresetPose(
    const std::shared_ptr<robot_interfaces::srv::SetPresetPose::Request> request,
    std::shared_ptr<robot_interfaces::srv::SetPresetPose::Response> response)
{
    RCLCPP_INFO(node_->get_logger(), "Received set_preset_pose request: '%s'", request->preset_name.c_str());
    response->success = controller_.moveToNamedTarget(request->preset_name);
}

void ArmDriverNode::spin()
{
    RCLCPP_INFO(node_->get_logger(), "arm_driver_node ready");

    rclcpp::Rate rate(1.0);
    while (rclcpp::ok())
    {
        rate.sleep();
    }
}

}  // namespace arm_control

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        arm_control::ArmDriverNode driver;
        driver.spin();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("arm_driver_node"), "arm_driver_node init failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
