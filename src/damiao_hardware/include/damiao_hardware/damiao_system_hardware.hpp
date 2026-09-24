#pragma once

#include <damiao_core/bus.hpp>
#include <hardware_interface/system_interface.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <thread>

namespace damiao_hardware
{

// 每轴的电机输出轴到关节坐标转换；角度和速度单位为 rad、rad/s。
struct AxisConfig
{
    std::string joint_name;
    std::string motor_name;
    std::string model;
    std::uint16_t esc_id = 0;
    std::uint16_t mst_id = 0;
    int direction = 0;
    double zero_offset_motor_output_rad = 0.0;
    double extra_reduction = 0.0;
    double min_position_rad = 0.0;
    double max_position_rad = 0.0;
    double max_velocity_rad_s = 0.0;
    double activation_position_tolerance_rad = 0.0;
};

// 一条 CAN 总线共用的管理、反馈和周期期限。
struct SystemConfig
{
    std::string can_interface;
    std::chrono::milliseconds feedback_timeout{0};
    std::chrono::milliseconds management_timeout{0};
    std::chrono::milliseconds management_quiet_period{0};
    std::chrono::milliseconds inactive_poll_period{0};
    std::chrono::milliseconds max_control_period{0};
    bool allow_enable_on_activate = false;
};

// 固定存储最多六轴的接口数值；控制周期整组验证，再交由核心库批量发送。
class DamiaoSystemHardware final : public hardware_interface::SystemInterface
{
public:
    ~DamiaoSystemHardware() override;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // 管理事务仅在生命周期线程或失能查询线程中执行；read/write 不查询寄存器。
    void start_inactive_polling();
    void stop_inactive_polling();
    bool disable_all();
    void close_bus();
    bool decode_state(std::size_t index, const damiao::MotorState& state,
        double& position, double& velocity) const;
    double motor_position(std::size_t index, double joint_position) const;

    SystemConfig config_;
    std::array<AxisConfig, damiao::max_motors> axes_{};
    std::size_t axis_count_ = 0;
    std::unique_ptr<damiao::DamiaoBus> bus_;
    std::thread inactive_poller_;
    std::atomic<bool> stop_poller_{true};
    std::array<double, damiao::max_motors> position_state_{};
    std::array<double, damiao::max_motors> velocity_state_{};
    std::array<double, damiao::max_motors> position_command_{};
    std::array<double, damiao::max_motors> last_command_{};
    std::array<damiao::Deadline, damiao::max_motors> last_state_received_{};
    // 用最近一次整组发送的时刻衡量控制间隔，避免激活事务阻塞控制循环时误判周期。
    damiao::Deadline last_command_sent_at_{};
    bool initial_hold_pending_ = false;
    bool configured_ = false;
    bool active_ = false;
    bool enable_attempted_ = false;
    bool fault_ = false;
};

}  // namespace damiao_hardware
