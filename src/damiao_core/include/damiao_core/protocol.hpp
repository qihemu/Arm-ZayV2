#pragma once

#include <damiao_core/types.hpp>

namespace damiao
{

// 通用命令独立编码，不在连接、清错或退出时隐式执行。
enum class ManagementCommand : std::uint8_t
{
    ClearError = 0xFB,
    Enable = 0xFC,
    Disable = 0xFD,
    SaveZero = 0xFE
};

enum class RegisterOperation : std::uint8_t
{
    Read = 0x33,
    Write = 0x55
};

// 解码调用者须提供期望字段；匹配字段不能消除协议本身全部回应歧义。
struct RegisterReplyExpectation
{
    MotorAddress address;
    RegisterOperation operation = RegisterOperation::Read;
    std::uint8_t register_id = 0;
};

// 无状态的纯数据接口，可并发调用；不打开设备、不发报文、不创建线程。
// 校验标准帧、地址、长度、有限性、范围及寄存器类型，不静默限幅。
class DamiaoProtocol
{
public:
    // 校验 ESC_ID、MST_ID 是否落在经典 CAN 标准帧及协议允许范围内。
    static Status validate_address(const MotorAddress& address);
    // 校验位置、速度和力矩映射上限均为有限正值。
    static Status validate_mapping_limits(const MappingLimits& limits);
    // 按寄存器表校验写权限以及写入值的类型和取值约束。
    static Status validate_register_write(std::uint8_t register_id, const RegisterValue& value);
    // 将电机原始状态码转换为稳定的诊断文本。
    static const char* status_description(std::uint8_t raw_status) noexcept;
    // 将输出轴位置和最大速度命令编码为位置速度控制帧。
    static Result<CanFrame> encode_position_velocity(
        std::uint16_t esc_id, const PositionVelocityCommand& command);

    // 解码普通反馈，并按完整 MST_ID 和数据低位 ESC_ID 核对目标电机。
    // sequence 由总线层维护；mapping_revision 用于记录本次解码采用的映射语义。
    static Result<MotorState> decode_feedback(
        const CanFrame& frame, const MotorAddress& address,
        const MappingLimits& limits, std::uint64_t mapping_revision = 0);

    // 编码指定寄存器的读取请求帧。
    static Result<CanFrame> encode_read_register(
        std::uint16_t esc_id, std::uint8_t register_id);
    // 校验寄存器元数据并编码指定寄存器的写入请求帧。
    static Result<CanFrame> encode_write_register(
        std::uint16_t esc_id, std::uint8_t register_id, const RegisterValue& value);
    // 校验地址、操作码和寄存器号后解码寄存器响应值。
    static Result<RegisterValue> decode_register_reply(
        const CanFrame& frame, const RegisterReplyExpectation& expected);

    // 编码独立状态查询帧，避免通过发送运动目标获取反馈。
    static Result<CanFrame> encode_state_query(std::uint16_t esc_id);
    // 按电机控制模式编码使能、失能、清错或保存零点命令。
    static Result<CanFrame> encode_management_command(
        std::uint16_t esc_id, ControlMode mode, ManagementCommand command);
    // 编码保存参数帧；失能许可与期限由管理层负责。
    static Result<CanFrame> encode_save_parameters(std::uint16_t esc_id);
    // 校验保存参数响应是否来自指定电机且执行成功。
    static Status decode_save_reply(const CanFrame& frame, const MotorAddress& address);
};

}  // namespace damiao
