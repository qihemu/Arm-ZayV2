#pragma once
#include <damiao_core/h55/types.hpp>
#include <damiao_core/protocol.hpp>
namespace damiao::h55
{
// [H55专用] 不改变DamiaoProtocol原位置模式函数；只复用确认一致的反馈解码。
Result<CanFrame> encode_velocity(std::uint16_t esc_id, double radians_per_second);
Result<CanFrame> encode_read(std::uint16_t esc_id, std::uint8_t register_id);
Result<RegisterValue> decode_register(const CanFrame &frame, MotorAddress address, std::uint8_t rid);
// Only volatile safety registers are writable through this H55 entry point.
Result<CanFrame> encode_protection_write(std::uint16_t esc_id, std::uint8_t rid, double value);
bool is_register_frame(const CanFrame &frame) noexcept;
const char *status_description(std::uint8_t status) noexcept;
} // namespace damiao::h55
