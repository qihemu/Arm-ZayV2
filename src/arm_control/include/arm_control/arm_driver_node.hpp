#pragma once

#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/srv/arm_move_to_point.hpp>
#include <robot_interfaces/srv/set_preset_pose.hpp>

#include "arm_control/arm_controller.hpp"

namespace arm_control
{

class ArmDriverNode
{
public:
    ArmDriverNode();
    void spin();

private:
    void setupServices();
    void handleMoveToPoint(
        const std::shared_ptr<robot_interfaces::srv::ArmMoveToPoint::Request> request,
        std::shared_ptr<robot_interfaces::srv::ArmMoveToPoint::Response> response);
    void handleSetPresetPose(
        const std::shared_ptr<robot_interfaces::srv::SetPresetPose::Request> request,
        std::shared_ptr<robot_interfaces::srv::SetPresetPose::Response> response);

    rclcpp::Node::SharedPtr node_;
    ArmController controller_;
    rclcpp::Service<robot_interfaces::srv::ArmMoveToPoint>::SharedPtr move_to_point_service_;
    rclcpp::Service<robot_interfaces::srv::SetPresetPose>::SharedPtr set_preset_pose_service_;
};

}  // namespace arm_control
