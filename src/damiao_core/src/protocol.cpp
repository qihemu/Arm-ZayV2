#include <damiao_core/protocol.hpp>

namespace damiao
{
namespace
{

// 所有协议占位实现均返回空结果，避免把未实现路径伪装成成功。
template<typename T>
Result<T> unsupported_protocol_result()
{
    return {{ErrorCode::Unsupported, "Damiao protocol is not implemented yet."}, std::nullopt};
}

}  // namespace

Result<CanFrame> DamiaoProtocol::encode_position_velocity(
    std::uint16_t, const PositionVelocityCommand&)
{
    return unsupported_protocol_result<CanFrame>();
}

Result<MotorState> DamiaoProtocol::decode_feedback(
    const CanFrame&, const MotorAddress&, const MappingLimits&, std::uint64_t)
{
    return unsupported_protocol_result<MotorState>();
}

Result<CanFrame> DamiaoProtocol::encode_read_register(std::uint16_t, std::uint8_t)
{
    return unsupported_protocol_result<CanFrame>();
}

Result<CanFrame> DamiaoProtocol::encode_write_register(
    std::uint16_t, std::uint8_t, const RegisterValue&)
{
    return unsupported_protocol_result<CanFrame>();
}

Result<RegisterValue> DamiaoProtocol::decode_register_reply(
    const CanFrame&, const RegisterReplyExpectation&)
{
    return unsupported_protocol_result<RegisterValue>();
}

Result<CanFrame> DamiaoProtocol::encode_state_query(std::uint16_t)
{
    return unsupported_protocol_result<CanFrame>();
}

Result<CanFrame> DamiaoProtocol::encode_management_command(
    std::uint16_t, ControlMode, ManagementCommand)
{
    return unsupported_protocol_result<CanFrame>();
}

Result<CanFrame> DamiaoProtocol::encode_save_parameters(std::uint16_t)
{
    return unsupported_protocol_result<CanFrame>();
}

}  // namespace damiao
