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
// 当前所有方法立即返回 Unsupported 且 value 为空，不产生成功帧或反馈。
// 后续实现需校验帧格式、地址、长度、有限性、范围及寄存器类型。
class DamiaoProtocol
{
public:
    static Result<CanFrame> encode_position_velocity(
        std::uint16_t esc_id, const PositionVelocityCommand& command);

    // 后续普通反馈解码按完整 MST_ID 和数据低位 ESC_ID 核对目标电机。
    // sequence 由未来总线层维护；本接口接收映射版本以记录解码语义。
    static Result<MotorState> decode_feedback(
        const CanFrame& frame, const MotorAddress& address,
        const MappingLimits& limits, std::uint64_t mapping_revision = 0);

    static Result<CanFrame> encode_read_register(
        std::uint16_t esc_id, std::uint8_t register_id);
    static Result<CanFrame> encode_write_register(
        std::uint16_t esc_id, std::uint8_t register_id, const RegisterValue& value);
    static Result<RegisterValue> decode_register_reply(
        const CanFrame& frame, const RegisterReplyExpectation& expected);

    // 独立状态查询帧，避免通过发送运动目标获取反馈。
    static Result<CanFrame> encode_state_query(std::uint16_t esc_id);
    static Result<CanFrame> encode_management_command(
        std::uint16_t esc_id, ControlMode mode, ManagementCommand command);
    // 仅编码存参数帧；失能许可、期限和完成确认属于未来管理层。
    static Result<CanFrame> encode_save_parameters(std::uint16_t esc_id);
};

}  // namespace damiao
