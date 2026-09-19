#pragma once

#include <damiao_core/types.hpp>

namespace damiao
{

enum class RegisterType
{
    Float,
    UInt32
};

// 仅列已核对寄存器，不对保留地址猜测类型或写权限。
struct RegisterInfo
{
    std::uint8_t id;
    const char* name;
    RegisterType type;
    bool writable;
};

// 查找已核对的寄存器元数据；未知或保留地址返回 nullptr。
const RegisterInfo* register_info(std::uint8_t register_id) noexcept;

// M3/M4 维护路径常用寄存器 ID，避免调用方硬编码 RID。
namespace RegisterId
{
constexpr std::uint8_t MstId = 0x07;
constexpr std::uint8_t EscId = 0x08;
constexpr std::uint8_t Timeout = 0x09;
constexpr std::uint8_t CtrlMode = 0x0A;
constexpr std::uint8_t Pmax = 0x15;
constexpr std::uint8_t Vmax = 0x16;
constexpr std::uint8_t Tmax = 0x17;
constexpr std::uint8_t SoftwareVersion = 0x0E;
}  // namespace RegisterId

}  // namespace damiao
