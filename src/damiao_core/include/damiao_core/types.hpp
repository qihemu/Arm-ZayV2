#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>

namespace damiao
{

using SteadyClock = std::chrono::steady_clock;
using Deadline = SteadyClock::time_point;

// 枚举值为控制模式寄存器编码，CAN ID 偏移由协议层计算。
enum class ControlMode : std::uint32_t
{
    Mit = 1,
    PositionVelocity = 2,
    Velocity = 3,
    PositionCurrentLimit = 4
};

// 地址为标准 11-bit ID；后续协议实现将限制 ESC_ID 为 1～15。
struct MotorAddress
{
    std::uint16_t esc_id = 0;
    std::uint16_t mst_id = 0;
};

// 只表达经典 CAN 标准数据帧；有效载荷长度为 0～8，不是系统调用字节数。
// 扩展帧、RTR、错误帧和 CAN FD 不可作为普通电机帧传入协议层。
struct CanFrame
{
    std::uint16_t id = 0;
    std::uint8_t length = 0;
    std::array<std::uint8_t, 8> data{};
    SteadyClock::time_point received_at{};
};

// 三个有限正值是逐电机编解码范围，不代表机械限位或持续输出能力。
struct MappingLimits
{
    double position_rad = 0.0;
    double velocity_rad_s = 0.0;
    double torque_nm = 0.0;
};

// 全部运动量采用电机减速器输出轴坐标，不重复折算内置减速比。
// received_at 为主机接收时刻，非设备采样时刻；valid=false 表示反馈无效。
struct MotorState
{
    double output_position_rad = 0.0;
    double output_velocity_rad_s = 0.0;
    // 电机报告的估计力矩，未经校核不能当作关节外力矩传感器值。
    double reported_torque_nm = 0.0;
    std::uint8_t raw_status = 0;
    std::uint8_t mos_temperature_c = 0;
    std::uint8_t rotor_temperature_c = 0;
    SteadyClock::time_point received_at{};
    std::uint64_t sequence = 0;
    std::uint64_t mapping_revision = 0;
    bool valid = false;
};

// 速度字段是最大绝对速度，不是带符号的轨迹速度。
struct PositionVelocityCommand
{
    double output_position_rad = 0.0;
    double max_output_speed_rad_s = 0.0;
};

enum class ErrorCode
{
    Ok,
    InvalidConfiguration,
    InvalidCommand,
    Timeout,
    Disconnected,
    BusError,
    StaleFeedback,
    MotorFault,
    OwnershipConflict,
    AmbiguousReply,
    PartialFailure,
    Unsupported
};

// 文本仅供低频诊断；本骨架不承诺无分配或硬实时行为。
struct Status
{
    ErrorCode code = ErrorCode::Ok;
    std::string message;
};

// 成功值须同时满足 status.code == Ok 和 value.has_value()。
template<typename T>
struct Result
{
    Status status;
    std::optional<T> value;
};

// 寄存器类型以后续新版手册表为准，不通过数值大小猜测。
using RegisterValue = std::variant<float, std::uint32_t>;

}  // namespace damiao
