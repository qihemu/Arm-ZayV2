#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/way_point_info.hpp>
#include <robot_interfaces/srv/arm_move_to_point.hpp>

namespace arm_control
{

// 基于 MoveIt 的机械臂控制封装类
class ArmController
{
public:
    explicit ArmController(const rclcpp::Node::SharedPtr& node);
    ~ArmController();

    // 初始化 MoveGroupInterface 与执行器
    void initialize();

    // 移动到 SRDF 中定义的命名姿态（如 home、init）
    bool moveToNamedTarget(const std::string& target_name);
    // 移动到目标笛卡尔位姿
    bool moveToPose(const geometry_msgs::msg::Pose& target_pose);
    // 根据 WayPointInfo 移动到目标点
    bool moveToWayPoint(const robot_interfaces::msg::WayPointInfo& waypoint_info);
    // 移动到目标关节角度
    bool moveToJointValues(const std::vector<double>& joint_values);
    // 规划并执行当前目标
    bool planAndExecute();

    // 获取底层 MoveGroupInterface，便于扩展高级功能
    moveit::planning_interface::MoveGroupInterface& moveGroup();
    const moveit::planning_interface::MoveGroupInterface& moveGroup() const;

private:
    // 从参数服务器加载配置
    void loadParameters();
    // 启动独立线程 spin 节点（MoveIt 接口要求）
    void startExecutor();
    // 停止执行器线程
    void stopExecutor();
    // 注册 ROS 服务
    void setupServices();
    // 处理移动到目标点服务请求
    void handleMoveToPoint(
        const std::shared_ptr<robot_interfaces::srv::ArmMoveToPoint::Request> request,
        std::shared_ptr<robot_interfaces::srv::ArmMoveToPoint::Response> response);

    rclcpp::Node::SharedPtr node_;
    std::string planning_group_;              // 规划组名称
    std::string robot_description_name_;         // 机器人 URDF 参数名
    double planning_time_{5.0};                // 规划超时时间（秒）
    double max_velocity_scaling_factor_{0.1};  // 最大速度缩放系数
    double max_acceleration_scaling_factor_{0.1};  // 最大加速度缩放系数

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Service<robot_interfaces::srv::ArmMoveToPoint>::SharedPtr move_to_point_service_;

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread executor_thread_;
    std::atomic<bool> spinning_{false};
};

}  // namespace arm_control
