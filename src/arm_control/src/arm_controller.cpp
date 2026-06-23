#include <algorithm>
#include <memory>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "arm_control/arm_controller.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>
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
    cartesian_step_size_ = node_->declare_parameter<double>("cartesian_step_size", 0.01);
    cartesian_jump_threshold_ = node_->declare_parameter<double>("cartesian_jump_threshold", 0.01);
    cartesian_min_fraction_ = node_->declare_parameter<double>("cartesian_min_fraction", 0.95);
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

    // 提前启动关节状态监听，避免运动时再订阅导致阻塞
    if (!move_group_->startStateMonitor(10.0))
    {
        RCLCPP_WARN(node_->get_logger(), "joint_states not available yet, will retry during motion");
    }

    RCLCPP_INFO(node_->get_logger(), "ArmController initialized for planning group '%s'", planning_group_.c_str());
}

void ArmController::startExecutor()
{
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
    executor_->add_node(node_);
    spinning_ = true;
    executor_thread_ = std::thread([this]() {
        while (spinning_ && rclcpp::ok())
        {
            executor_->spin_some();
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
    executor_.reset();
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

    double velocity_scale = max_velocity_scaling_factor_;
    double acceleration_scale = max_acceleration_scaling_factor_;
    if (waypoint_info.velocity > 0.0f)
    {
        velocity_scale = std::clamp(static_cast<double>(waypoint_info.velocity), 0.01, 1.0);
    }
    if (waypoint_info.acceleration > 0.0f)
    {
        acceleration_scale = std::clamp(static_cast<double>(waypoint_info.acceleration), 0.01, 1.0);
    }

    return moveToPoseCartesian(target_pose, velocity_scale, acceleration_scale);
}

bool ArmController::moveToNamedTarget(const std::string& target_name)
{
    std::lock_guard<std::mutex> lock(motion_mutex_);
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

bool ArmController::moveToPoseCartesian(const geometry_msgs::msg::Pose& target_pose,
                                        double velocity_scale, double acceleration_scale)
{
    std::lock_guard<std::mutex> lock(motion_mutex_);

    move_group_->setPoseReferenceFrame(move_group_->getPlanningFrame());
    move_group_->setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(
        waypoints, cartesian_step_size_, cartesian_jump_threshold_, trajectory, true);

    if (fraction < cartesian_min_fraction_)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "Cartesian path planning achieved only %.1f%%, required %.1f%%",
                     fraction * 100.0, cartesian_min_fraction_ * 100.0);
        return false;
    }

    if (trajectory.joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "Cartesian path is empty");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Cartesian path planned: %.1f%%", fraction * 100.0);

    robot_trajectory::RobotTrajectory robot_traj(move_group_->getRobotModel(), move_group_->getName());

    // 优先使用当前关节状态；若获取失败则从轨迹首点构造，避免空指针崩溃
    moveit::core::RobotStatePtr current_state = move_group_->getCurrentState(5.0);
    if (current_state)
    {
        robot_traj.setRobotTrajectoryMsg(*current_state, trajectory);
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "Failed to get current state, using trajectory start point");
        moveit::core::RobotState start_state(move_group_->getRobotModel());
        const moveit::core::JointModelGroup* joint_model_group =
            move_group_->getRobotModel()->getJointModelGroup(move_group_->getName());
        start_state.setJointGroupPositions(joint_model_group,
                                           trajectory.joint_trajectory.points.front().positions);
        robot_traj.setRobotTrajectoryMsg(start_state, trajectory);
    }

    trajectory_processing::IterativeParabolicTimeParameterization time_param;
    if (!time_param.computeTimeStamps(robot_traj, velocity_scale, acceleration_scale))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to compute time stamps for Cartesian path");
        return false;
    }

    robot_traj.getRobotTrajectoryMsg(trajectory);

    moveit::core::MoveItErrorCode result = move_group_->execute(trajectory);
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to execute Cartesian path, error code: %d", result.val);
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "Cartesian move completed to [%.3f, %.3f, %.3f]",
                target_pose.position.x, target_pose.position.y, target_pose.position.z);
    return true;
}

bool ArmController::moveToPose(const geometry_msgs::msg::Pose& target_pose)
{
    std::lock_guard<std::mutex> lock(motion_mutex_);
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
    std::lock_guard<std::mutex> lock(motion_mutex_);
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
    std::lock_guard<std::mutex> lock(motion_mutex_);
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
