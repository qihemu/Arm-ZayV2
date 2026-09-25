#pragma once
#include <cstdint>
namespace damiao::h55
{
// [H55专用] 只开放经手册/实测核验的字段，控制程序没有任意寄存器写入口。
struct RegisterInfo
{
    std::uint8_t id;
    const char *name;
    bool integer;
};
const RegisterInfo *register_info(std::uint8_t id) noexcept;
} // namespace damiao::h55
