#include <algorithm>
#include <functional>
#include <memory>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "arm_control/arm_driver_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace arm_control
{

ArmController::ArmController(const rclcpp::Node::SharedPtr& node) : node_(node)
{
    loadParameters();
}

ArmController::~ArmController()
{
    stopExecutor();
}

void ArmController::loadParameters()
{
    planning_group_ = node_->declare_parameter<std::string>("planning_group", "arm");
    robot_description_name_ = node_->declare_parameter<std::string>("robot_description_name", "robot_description");
    planning_time_ = node_->declare_parameter<double>("planning_time", 5.0);
    max_velocity_scaling_factor_ = node_->declare_parameter<double>("max_velocity_scaling_factor", 0.5);
    max_acceleration_scaling_factor_ =
        node_->declare_parameter<double>("max_acceleration_scaling_factor", 0.5);
}

void ArmController::initialize()
{
    startExecutor();

    // 创建 MoveGroupInterface，连接 move_group 节点
    moveit::planning_interface::MoveGroupInterface::Options options(planning_group_, robot_description_name_);
    try
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, options);
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(node_->get_logger(), "Failed to create MoveGroupInterface: %s", e.what());
        throw;
    }

    // 设置规划参数
    move_group_->setPlanningTime(planning_time_);
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);

    setupServices();

    RCLCPP_INFO(node_->get_logger(), "ArmController initialized for planning group '%s'", planning_group_.c_str());
}

void ArmController::setupServices()
{
    move_to_point_service_ = node_->create_service<robot_interfaces::srv::ArmMoveToPoint>(
        "/arm/move_to_point",
        std::bind(&ArmController::handleMoveToPoint, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(node_->get_logger(), "Service /arm/move_to_point ready");
}


void ArmController::handleMoveToPoint(
    const std::shared_ptr<robot_interfaces::srv::ArmMoveToPoint::Request> request,
    std::shared_ptr<robot_interfaces::srv::ArmMoveToPoint::Response> response)
{
    RCLCPP_INFO(node_->get_logger(), "Received move_to_point request");
    response->success = moveToWayPoint(request->target_point);
}

void ArmController::startExecutor()
{
    executor_.add_node(node_);
    spinning_ = true;
    executor_thread_ = std::thread([this]() {
        while (spinning_ && rclcpp::ok())
        {
            executor_.spin_some();
        }
    });
}

void ArmController::stopExecutor()
{
    spinning_ = false;
    if (executor_thread_.joinable())
    {
        executor_thread_.join();
    }
}

bool ArmController::moveToWayPoint(const robot_interfaces::msg::WayPointInfo& waypoint_info)
{
    if (waypoint_info.waypoint.size() != 6)
    {
        RCLCPP_ERROR(node_->get_logger(), "Invalid waypoint size: %zu, expected 6", waypoint_info.waypoint.size());
        return false;
    }

    // waypoint: [x, y, z, roll, pitch, yaw]，单位 m / rad
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = waypoint_info.waypoint[0];
    target_pose.position.y = waypoint_info.waypoint[1];
    target_pose.position.z = waypoint_info.waypoint[2];

    tf2::Quaternion quaternion;
    quaternion.setRPY(waypoint_info.waypoint[3], waypoint_info.waypoint[4], waypoint_info.waypoint[5]);
    target_pose.orientation = tf2::toMsg(quaternion);

    // 请求中指定了速度/加速度时，作为缩放系数使用（取值范围 0~1）
    if (waypoint_info.velocity > 0.0f)
    {
        move_group_->setMaxVelocityScalingFactor(
            std::clamp(static_cast<double>(waypoint_info.velocity), 0.01, 1.0));
    }
    else
    {
        move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
    }

    if (waypoint_info.acceleration > 0.0f)
    {
        move_group_->setMaxAccelerationScalingFactor(
            std::clamp(static_cast<double>(waypoint_info.acceleration), 0.01, 1.0));
    }
    else
    {
        move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);
    }

    return moveToPose(target_pose);
}

bool ArmController::moveToNamedTarget(const std::string& target_name)
{
    move_group_->setNamedTarget(target_name);
    moveit::core::MoveItErrorCode result = move_group_->move();
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to named target '%s', error code: %d",
                     target_name.c_str(), result.val);
        return false;
    }
    return true;
}

bool ArmController::moveToPose(const geometry_msgs::msg::Pose& target_pose)
{
    move_group_->setPoseTarget(target_pose);

    moveit::core::MoveItErrorCode result = move_group_->move();
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to target pose, error code: %d", result.val);
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "Moved to pose [%.3f, %.3f, %.3f]",
                target_pose.position.x, target_pose.position.y, target_pose.position.z);
    return true;
}

bool ArmController::moveToJointValues(const std::vector<double>& joint_values)
{
    if (joint_values.size() != move_group_->getActiveJoints().size())
    {
        RCLCPP_ERROR(node_->get_logger(), "Joint count mismatch: got %zu, expected %zu",
                     joint_values.size(), move_group_->getActiveJoints().size());
        return false;
    }

    move_group_->setJointValueTarget(joint_values);
    moveit::core::MoveItErrorCode result = move_group_->move();
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move to joint values, error code: %d", result.val);
        return false;
    }
    return true;
}

bool ArmController::planAndExecute()
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode plan_result = move_group_->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed, error code: %d", plan_result.val);
        return false;
    }

    moveit::core::MoveItErrorCode execute_result = move_group_->execute(plan);
    if (execute_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Execution failed, error code: %d", execute_result.val);
        return false;
    }
    return true;
}

moveit::planning_interface::MoveGroupInterface& ArmController::moveGroup()
{
    return *move_group_;
}

const moveit::planning_interface::MoveGroupInterface& ArmController::moveGroup() const
{
    return *move_group_;
}

}  // namespace arm_control



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // 创建控制节点
    auto node = rclcpp::Node::make_shared("arm_driver_node");

    // 初始化机械臂控制器
    arm_control::ArmController controller(node);
    try
    {
        controller.initialize();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(node->get_logger(), "arm_driver_node init failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "arm_driver_node ready");

    // 保持节点运行
    rclcpp::Rate rate(1.0);
    while (rclcpp::ok())
    {
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
