#include <damiao_core/protocol.hpp>
#include <damiao_core/registers.hpp>

#include <cmath>
#include <cstring>
#include <limits>

namespace damiao
{
namespace
{

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
    "Damiao protocol requires IEEE-754 binary32 floats.");

template<typename T>
// 构造不含结果值的统一失败返回对象。
Result<T> failure(ErrorCode code, const char* message)
{
    return {{code, message}, std::nullopt};
}

// 检查 ESC_ID 是否满足首版协议的 1～15 范围。
bool valid_esc(std::uint32_t id)
{
    return id >= 1 && id <= 15;
}

// 检查标准帧 ID 和协议期望的固定有效载荷长度。
bool valid_frame(const CanFrame& frame, std::uint8_t length)
{
    return frame.id <= 0x7FF && frame.length == length;
}

// 字节序显式处理，float 经 memcpy 转换，避免未对齐访问与严格别名问题。
std::uint32_t load_u32(const std::uint8_t* data)
{
    return std::uint32_t(data[0]) | (std::uint32_t(data[1]) << 8)
        | (std::uint32_t(data[2]) << 16) | (std::uint32_t(data[3]) << 24);
}

// 将主机整数显式写为协议规定的小端字节序。
void store_u32(std::uint8_t* data, std::uint32_t value)
{
    for (unsigned int index = 0; index < 4; ++index)
    {
        data[index] = static_cast<std::uint8_t>(value >> (8 * index));
    }
}

// 从小端 IEEE-754 字节序安全读取单精度值。
float load_float(const std::uint8_t* data)
{
    const auto bits = load_u32(data);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

// 将单精度值按 IEEE-754 位模式安全写入协议缓冲区。
void store_float(std::uint8_t* data, float value)
{
    std::uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    store_u32(data, bits);
}

// 确认 double 命令可无溢出且不下溢为零地传入 float 协议字段。
bool float_representable(double value)
{
    return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max()
        && (value == 0.0 || static_cast<float>(value) != 0.0F);
}

// 构造寄存器和管理事务共用的参数帧头。
CanFrame parameter_frame(std::uint16_t esc, std::uint8_t operation, std::uint8_t rid,
    std::uint8_t length)
{
    CanFrame frame;
    frame.id = 0x7FF;
    frame.length = length;
    frame.data[0] = static_cast<std::uint8_t>(esc);
    frame.data[1] = static_cast<std::uint8_t>(esc >> 8);
    frame.data[2] = operation;
    frame.data[3] = rid;
    return frame;
}

// 核对参数响应的完整地址、操作码和寄存器号。
bool matches_reply(const CanFrame& frame, const MotorAddress& address,
    std::uint8_t operation, std::uint8_t rid)
{
    return frame.id == address.mst_id && frame.data[0] == address.esc_id
        && frame.data[1] == 0 && frame.data[2] == operation && frame.data[3] == rid;
}

// 先计算归一化比例，避免 2*limit 溢出；量化误差按各字段步长解释。
double unmap(std::uint32_t value, std::uint32_t maximum, double limit)
{
    return (2.0 * (static_cast<double>(value) / maximum) - 1.0) * limit;
}

}  // namespace

Status DamiaoProtocol::validate_address(const MotorAddress& address)
{
    if (!valid_esc(address.esc_id) || address.mst_id > 0x7FE
        || (address.mst_id & 0xFF) == address.esc_id)
    {
        return {ErrorCode::InvalidConfiguration, "Invalid or conflicting ESC_ID/MST_ID."};
    }
    return {};
}

Status DamiaoProtocol::validate_mapping_limits(const MappingLimits& limits)
{
    if (!std::isfinite(limits.position_rad) || limits.position_rad <= 0.0
        || !std::isfinite(limits.velocity_rad_s) || limits.velocity_rad_s <= 0.0
        || !std::isfinite(limits.torque_nm) || limits.torque_nm <= 0.0)
    {
        return {ErrorCode::InvalidConfiguration, "PMAX/VMAX/TMAX must be finite positive values."};
    }
    return {};
}

Status DamiaoProtocol::validate_register_write(std::uint8_t rid, const RegisterValue& value)
{
    const auto* info = register_info(rid);
    if (info == nullptr || !info->writable)
    {
        return {ErrorCode::InvalidCommand, "Unknown or read-only register."};
    }
    if (info->type == RegisterType::UInt32)
    {
        const auto* number = std::get_if<std::uint32_t>(&value);
        if (number == nullptr || (rid == 0x07 && *number > 0x7FE)
            || (rid == 0x08 && !valid_esc(*number))
            || (rid == 0x0A && (*number < 1 || *number > 4))
            || (rid == 0x23 && *number > 4))
        {
            return {ErrorCode::InvalidCommand, "Incorrect integer register type or range."};
        }
        return {};
    }
    const auto* number = std::get_if<float>(&value);
    if (number == nullptr || !std::isfinite(*number))
    {
        return {ErrorCode::InvalidCommand, "Register requires a finite float."};
    }
    const float x = *number;
    bool allowed = true;
    // 仅使用手册明示范围；OV_Value 的 TBD 不能作为可写安全范围。
    switch (rid)
    {
        case 0x00: allowed = x > 10.0F; break;
        case 0x01:
        case 0x19:
        case 0x1A:
        case 0x1B:
        case 0x1C: allowed = x >= 0.0F; break;
        case 0x02: allowed = x >= 80.0F && x < 200.0F; break;
        case 0x03: allowed = x > 0.0F && x < 1.0F; break;
        case 0x04: allowed = x > 0.0F && x < std::numeric_limits<float>::max(); break;
        case 0x05: allowed = x < 0.0F; break;
        case 0x06:
        case 0x15:
        case 0x16:
        case 0x17: allowed = x > 0.0F; break;
        case 0x18:
        case 0x21: allowed = x >= 100.0F && x <= 10000.0F; break;
        case 0x1D: return {ErrorCode::Unsupported, "OV_Value range is unspecified by the manual."};
        case 0x1E: allowed = x > 0.0F && x <= 1.0F; break;
        case 0x1F: allowed = x >= 1.0F && x <= 30.0F; break;
        case 0x20: allowed = x > 0.0F && x < 500.0F; break;
        case 0x22: allowed = x > 0.0F && x <= 10000.0F; break;
        default: allowed = false; break;
    }
    return allowed ? Status{} : Status{ErrorCode::InvalidCommand, "Float register is out of range."};
}

const char* DamiaoProtocol::status_description(std::uint8_t status) noexcept
{
    switch (status)
    {
        case 0: return "Disabled";
        case 1: return "Enabled";
        case 3: return "Output encoder calibration error";
        case 4: return "Sensor output error";
        case 5: return "Motor encoder calibration error";
        case 8: return "Overvoltage";
        case 9: return "Undervoltage";
        case 10: return "Overcurrent";
        case 11: return "MOS overtemperature";
        case 12: return "Winding overtemperature";
        case 13: return "Communication lost";
        case 14: return "Overload";
        default: return "Unknown motor status";
    }
}

Result<CanFrame> DamiaoProtocol::encode_position_velocity(
    std::uint16_t esc, const PositionVelocityCommand& command)
{
    if (!valid_esc(esc) || !float_representable(command.output_position_rad)
        || !float_representable(command.max_output_speed_rad_s)
        || command.max_output_speed_rad_s < 0.0)
    {
        return failure<CanFrame>(ErrorCode::InvalidCommand, "Invalid position/absolute speed command.");
    }
    CanFrame frame;
    frame.id = esc + 0x100;
    frame.length = 8;
    store_float(frame.data.data(), static_cast<float>(command.output_position_rad));
    store_float(frame.data.data() + 4, static_cast<float>(command.max_output_speed_rad_s));
    return {{}, frame};
}

Result<MotorState> DamiaoProtocol::decode_feedback(const CanFrame& frame,
    const MotorAddress& address, const MappingLimits& limits, std::uint64_t revision)
{
    if (validate_address(address).code != ErrorCode::Ok
        || validate_mapping_limits(limits).code != ErrorCode::Ok)
    {
        return failure<MotorState>(ErrorCode::InvalidConfiguration, "Invalid address or mapping limits.");
    }
    if (!valid_frame(frame, 8) || frame.id != address.mst_id
        || (frame.data[0] & 0x0F) != address.esc_id)
    {
        return failure<MotorState>(ErrorCode::InvalidFrame, "Feedback length or motor address mismatch.");
    }
    MotorState state;
    // 三个压缩整数分别按 16、12、12 位映射回对称的输出轴物理量。
    state.output_position_rad = unmap((std::uint32_t(frame.data[1]) << 8) | frame.data[2],
        65535, limits.position_rad);
    state.output_velocity_rad_s = unmap((std::uint32_t(frame.data[3]) << 4) | (frame.data[4] >> 4),
        4095, limits.velocity_rad_s);
    state.reported_torque_nm = unmap((std::uint32_t(frame.data[4] & 0x0F) << 8) | frame.data[5],
        4095, limits.torque_nm);
    state.raw_status = frame.data[0] >> 4;
    state.mos_temperature_c = frame.data[6];
    state.rotor_temperature_c = frame.data[7];
    state.received_at = frame.received_at;
    state.mapping_revision = revision;
    state.valid = true;
    // 故障仍可携带有效测量；原始状态用于上层运行门控。
    return {{}, state};
}

Result<CanFrame> DamiaoProtocol::encode_read_register(std::uint16_t esc, std::uint8_t rid)
{
    if (!valid_esc(esc) || register_info(rid) == nullptr)
    {
        return failure<CanFrame>(ErrorCode::InvalidCommand, "Invalid ESC_ID or unknown register.");
    }
    return {{}, parameter_frame(esc, 0x33, rid, 4)};
}

Result<CanFrame> DamiaoProtocol::encode_write_register(
    std::uint16_t esc, std::uint8_t rid, const RegisterValue& value)
{
    const auto status = validate_register_write(rid, value);
    if (!valid_esc(esc) || status.code != ErrorCode::Ok)
    {
        return {status.code == ErrorCode::Ok ? Status{ErrorCode::InvalidCommand, "Invalid ESC_ID."}
            : status, std::nullopt};
    }
    auto frame = parameter_frame(esc, 0x55, rid, 8);
    // 寄存器表决定联合类型，编码时不根据数值大小猜测字段格式。
    if (const auto* number = std::get_if<float>(&value))
    {
        store_float(frame.data.data() + 4, *number);
    }
    else
    {
        store_u32(frame.data.data() + 4, std::get<std::uint32_t>(value));
    }
    return {{}, frame};
}

Result<RegisterValue> DamiaoProtocol::decode_register_reply(
    const CanFrame& frame, const RegisterReplyExpectation& expected)
{
    const auto* info = register_info(expected.register_id);
    if (validate_address(expected.address).code != ErrorCode::Ok || info == nullptr
        || (expected.operation != RegisterOperation::Read && expected.operation != RegisterOperation::Write))
    {
        return failure<RegisterValue>(ErrorCode::InvalidConfiguration, "Invalid reply expectation.");
    }
    if (!valid_frame(frame, 8) || !matches_reply(frame, expected.address,
        static_cast<std::uint8_t>(expected.operation), expected.register_id))
    {
        return failure<RegisterValue>(ErrorCode::InvalidFrame, "Register reply fields do not match.");
    }
    if (info->type == RegisterType::UInt32)
    {
        return {{}, RegisterValue{load_u32(frame.data.data() + 4)}};
    }
    const float value = load_float(frame.data.data() + 4);
    if (!std::isfinite(value))
    {
        return failure<RegisterValue>(ErrorCode::InvalidFrame, "Nonfinite register reply.");
    }
    return {{}, RegisterValue{value}};
}

Result<CanFrame> DamiaoProtocol::encode_state_query(std::uint16_t esc)
{
    if (!valid_esc(esc))
    {
        return failure<CanFrame>(ErrorCode::InvalidCommand, "Invalid ESC_ID.");
    }
    // 参考达妙 SocketCAN 例程的独立查询；固件 0xCC 支持仍须实测。
    return {{}, parameter_frame(esc, 0xCC, 0, 4)};
}

Result<CanFrame> DamiaoProtocol::encode_management_command(
    std::uint16_t esc, ControlMode mode, ManagementCommand command)
{
    const auto mode_code = static_cast<std::uint32_t>(mode);
    const auto code = static_cast<std::uint8_t>(command);
    if (!valid_esc(esc) || mode_code < 1 || mode_code > 4 || code < 0xFB || code > 0xFE)
    {
        return failure<CanFrame>(ErrorCode::InvalidCommand, "Invalid management command.");
    }
    CanFrame frame;
    frame.id = esc + static_cast<std::uint16_t>((mode_code - 1) * 0x100);
    frame.length = 8;
    frame.data.fill(0xFF);
    frame.data[7] = code;
    return {{}, frame};
}

Result<CanFrame> DamiaoProtocol::encode_save_parameters(std::uint16_t esc)
{
    if (!valid_esc(esc))
    {
        return failure<CanFrame>(ErrorCode::InvalidCommand, "Invalid ESC_ID.");
    }
    return {{}, parameter_frame(esc, 0xAA, 1, 4)};
}

Status DamiaoProtocol::decode_save_reply(const CanFrame& frame, const MotorAddress& address)
{
    if (validate_address(address).code != ErrorCode::Ok)
    {
        return {ErrorCode::InvalidConfiguration, "Invalid motor address."};
    }
    if ((frame.length != 4 && frame.length != 8) || frame.id > 0x7FF
        || !matches_reply(frame, address, 0xAA, 1))
    {
        return {ErrorCode::InvalidFrame, "Save acknowledgement does not match."};
    }
    return {};
}

}  // namespace damiao
