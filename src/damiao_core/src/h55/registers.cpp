#include <damiao_core/h55/registers.hpp>
namespace damiao::h55
{
const RegisterInfo *register_info(std::uint8_t id) noexcept
{
    static constexpr RegisterInfo registers[] = {
        {0x06, "MAX_SPD", false},  {0x07, "MST_ID", true}, {0x08, "ESC_ID", true},  {0x09, "TIMEOUT", true},
        {0x0A, "CTRL_MODE", true}, {0x0E, "sw_ver", true}, {0x15, "PMAX", false},   {0x16, "VMAX", false},
        {0x17, "TMAX", false},     {0x23, "can_br", true}, {0x24, "sub_ver", true}, {0x3C, "VBus", false},
        {0x3D, "Tpcb", false},     {0x3E, "Tmtr", false}};
    for (const auto &entry : registers)
    {
        if (entry.id == id)
        {
            return &entry;
        }
    }
    return nullptr;
}
} // namespace damiao::h55
