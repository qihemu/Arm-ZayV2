#include <damiao_core/registers.hpp>

namespace damiao
{
namespace
{

// J4310P V1.1 第 16～17 页、J4340 V1.2 第 17～18 页。
// 新版 dir 是 float，m_off 为 0x38，与旧例程不同。
constexpr RegisterInfo registers[] =
{
    {0x00, "UV_Value", RegisterType::Float, true},
    {0x01, "KT_Value", RegisterType::Float, true},
    {0x02, "OT_Value", RegisterType::Float, true},
    {0x03, "OC_Value", RegisterType::Float, true},
    {0x04, "ACC", RegisterType::Float, true},
    {0x05, "DEC", RegisterType::Float, true},
    {0x06, "MAX_SPD", RegisterType::Float, true},
    {0x07, "MST_ID", RegisterType::UInt32, true},
    {0x08, "ESC_ID", RegisterType::UInt32, true},
    {0x09, "TIMEOUT", RegisterType::UInt32, true},
    {0x0A, "CTRL_MODE", RegisterType::UInt32, true},
    {0x0B, "Damp", RegisterType::Float, false},
    {0x0C, "Inertia", RegisterType::Float, false},
    {0x0D, "hw_ver", RegisterType::UInt32, false},
    {0x0E, "sw_ver", RegisterType::UInt32, false},
    {0x0F, "SN", RegisterType::UInt32, false},
    {0x10, "NPP", RegisterType::UInt32, false},
    {0x11, "Rs", RegisterType::Float, false},
    {0x12, "Ls", RegisterType::Float, false},
    {0x13, "Flux", RegisterType::Float, false},
    {0x14, "Gr", RegisterType::Float, false},
    {0x15, "PMAX", RegisterType::Float, true},
    {0x16, "VMAX", RegisterType::Float, true},
    {0x17, "TMAX", RegisterType::Float, true},
    {0x18, "I_BW", RegisterType::Float, true},
    {0x19, "KP_ASR", RegisterType::Float, true},
    {0x1A, "KI_ASR", RegisterType::Float, true},
    {0x1B, "KP_APR", RegisterType::Float, true},
    {0x1C, "KI_APR", RegisterType::Float, true},
    {0x1D, "OV_Value", RegisterType::Float, true},
    {0x1E, "GREF", RegisterType::Float, true},
    {0x1F, "Deta", RegisterType::Float, true},
    {0x20, "V_BW", RegisterType::Float, true},
    {0x21, "IQ_c1", RegisterType::Float, true},
    {0x22, "VL_c1", RegisterType::Float, true},
    {0x23, "can_br", RegisterType::UInt32, true},
    {0x24, "sub_ver", RegisterType::UInt32, false},
    {0x25, "Boot_ver", RegisterType::UInt32, false},
    {0x37, "dir", RegisterType::Float, false},
    {0x38, "m_off", RegisterType::Float, false},
    {0x3B, "Imax", RegisterType::Float, false},
    {0x3C, "VBus", RegisterType::Float, false},
    {0x3D, "Tpcb", RegisterType::Float, false},
    {0x3E, "Tmtr", RegisterType::Float, false},
    {0x3F, "Iu_off", RegisterType::Float, false},
    {0x40, "Iv_off", RegisterType::Float, false},
    {0x41, "Iw_off", RegisterType::Float, false},
    {0x50, "p_m", RegisterType::Float, false},
    {0x51, "xout", RegisterType::Float, false},
};

}  // namespace

// 在线性小表中查找寄存器；未收录地址保持未知而非推断类型。
const RegisterInfo* register_info(std::uint8_t id) noexcept
{
    for (const auto& info : registers)
    {
        if (info.id == id)
        {
            return &info;
        }
    }
    return nullptr;
}

}  // namespace damiao
