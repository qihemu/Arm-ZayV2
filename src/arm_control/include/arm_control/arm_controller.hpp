#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/way_point_info.hpp>

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
    // 移动到目标笛卡尔位姿（OMPL 关节空间规划）
    bool moveToPose(const geometry_msgs::msg::Pose& target_pose);
    // 笛卡尔直线运动到目标位姿
    bool moveToPoseCartesian(const geometry_msgs::msg::Pose& target_pose,
                             double velocity_scale, double acceleration_scale);
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

    rclcpp::Node::SharedPtr node_;
    std::string planning_group_;                   // 规划组名称
    std::string robot_description_name_;           // 机器人 URDF 参数名
    double planning_time_{5.0};                    // 规划超时时间（秒）
    double max_velocity_scaling_factor_{0.1};      // 最大速度缩放系数
    double max_acceleration_scaling_factor_{0.1};  // 最大加速度缩放系数
    double cartesian_step_size_{0.01};             // 笛卡尔路径插值步长（m）
    double cartesian_jump_threshold_{0.0};         // 关节空间跳跃阈值（0 表示禁用）
    double cartesian_min_fraction_{0.95};          // 笛卡尔路径最低完成比例

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread executor_thread_;
    std::atomic<bool> spinning_{false};
    std::mutex motion_mutex_;  // 串行化运动请求，避免并发操作 move_group
};

}  // namespace arm_control
