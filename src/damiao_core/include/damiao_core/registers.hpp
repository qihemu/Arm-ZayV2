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

}  // namespace damiao
