#include <damiao_core/h55/protocol.hpp>
#include <damiao_core/h55/registers.hpp>
#include <cmath>
#include <cstring>
#include <limits>
namespace damiao::h55
{
Result<CanFrame> encode_velocity(std::uint16_t esc, double speed)
{
    if (esc < 1 || esc > 15 || !std::isfinite(speed) || std::abs(speed) > std::numeric_limits<float>::max())
    {
        return {{ErrorCode::InvalidCommand, "Invalid H55 velocity/address"}, std::nullopt};
    }
    CanFrame frame;
    frame.id = 0x200 + esc;
    frame.length = 4;
    const float value = static_cast<float>(speed);
    std::uint32_t bits;
    std::memcpy(&bits, &value, 4);
    for (int i = 0; i < 4; ++i)
    {
        frame.data[i] = (bits >> (8 * i)) & 0xff;
    }
    return {{}, frame};
}
Result<CanFrame> encode_read(std::uint16_t esc, std::uint8_t rid)
{
    if (esc < 1 || esc > 15 || register_info(rid) == nullptr)
    {
        return {{ErrorCode::InvalidCommand, "Unknown H55 register/address"}, std::nullopt};
    }
    CanFrame frame;
    frame.id = 0x7ff;
    frame.length = 8;
    frame.data[0] = esc;
    frame.data[2] = 0x33;
    frame.data[3] = rid;
    return {{}, frame};
}
Result<CanFrame> encode_protection_write(std::uint16_t esc, std::uint8_t rid, double value)
{
    if ((rid != 0x09 && rid != 0x06) || !std::isfinite(value) || value <= 0 ||
        (rid == 0x09 && (value > 20000 || std::floor(value) != value)) ||
        (rid == 0x06 && (value > std::numeric_limits<float>::max() || static_cast<float>(value) == 0)))
    {
        return {{ErrorCode::InvalidCommand, "Invalid H55 volatile protection write"}, std::nullopt};
    }
    auto frame = encode_read(esc, rid);
    if (!frame.value)
    {
        return frame;
    }
    frame.value->data[2] = 0x55;
    std::uint32_t bits = static_cast<std::uint32_t>(rid == 9 ? value : 0);
    if (rid == 6)
    {
        const float f = static_cast<float>(value);
        std::memcpy(&bits, &f, 4);
    }
    for (int i = 0; i < 4; ++i)
    {
        frame.value->data[i + 4] = (bits >> (8 * i)) & 0xff;
    }
    return frame;
}
bool is_register_frame(const CanFrame &f) noexcept
{
    return f.length == 8 && f.data[0] >= 1 && f.data[0] <= 15 && f.data[1] == 0 &&
           (f.data[2] == 0x33 || f.data[2] == 0x55 || f.data[2] == 0xaa);
}
Result<RegisterValue> decode_register(const CanFrame &f, MotorAddress address, std::uint8_t rid)
{
    const auto *info = register_info(rid);
    if (!info || f.id != address.mst_id || f.length != 8 || f.id > 0x7fe || f.data[0] != address.esc_id ||
        f.data[1] != 0 || f.data[2] != 0x33 || f.data[3] != rid)
    {
        return {{ErrorCode::InvalidFrame, "H55 register response mismatch"}, std::nullopt};
    }
    std::uint32_t bits = 0;
    for (int i = 0; i < 4; ++i)
    {
        bits |= std::uint32_t(f.data[i + 4]) << (8 * i);
    }
    if (info->integer)
    {
        return {{}, RegisterValue{bits}};
    }
    float value;
    std::memcpy(&value, &bits, 4);
    if (!std::isfinite(value))
    {
        return {{ErrorCode::InvalidFrame, "Nonfinite H55 register"}, std::nullopt};
    }
    return {{}, RegisterValue{value}};
}
const char *status_description(std::uint8_t code) noexcept
{
    switch (code)
    {
    case 0:
        return "Disabled";
    case 1:
        return "Enabled";
    case 2:
        return "Encoder fault";
    case 5:
        return "Calibration read fault";
    case 6:
        return "Motor parameter fault";
    case 7:
        return "Sensor fault";
    case 8:
        return "Overvoltage";
    case 9:
        return "Undervoltage";
    case 10:
        return "Overcurrent";
    case 11:
        return "Driver overtemperature";
    case 12:
        return "Motor overtemperature";
    case 13:
        return "Communication lost";
    case 14:
        return "Overload";
    default:
        return "Unknown H55 status";
    }
}
} // namespace damiao::h55
