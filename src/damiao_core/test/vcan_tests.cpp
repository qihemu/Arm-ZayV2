#include <damiao_core/transport.hpp>
#include <damiao_core/protocol.hpp>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace
{

// 虚拟电机用原始 Socket 回应，验证真实内核收发路径，不使用库内部往返转换。
class VirtualMotor
{
public:
    ~VirtualMotor()
    {
        if (descriptor_ >= 0)
        {
            ::close(descriptor_);
        }
    }

    bool open(const std::string& interface)
    {
        descriptor_ = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
        if (descriptor_ < 0)
        {
            return false;
        }
        struct ifreq request{};
        std::memcpy(request.ifr_name, interface.c_str(), interface.size() + 1);
        if (::ioctl(descriptor_, SIOCGIFINDEX, &request) != 0)
        {
            return false;
        }
        struct sockaddr_can address{};
        address.can_family = AF_CAN;
        address.can_ifindex = request.ifr_ifindex;
        return ::bind(descriptor_, reinterpret_cast<struct sockaddr*>(&address), sizeof(address)) == 0;
    }

    bool reply()
    {
        can_frame response{};
        response.can_id = 0x11;
        response.len = 8;
        const std::uint8_t bytes[] = {0x01, 0x80, 0, 0x80, 0, 0, 30, 40};
        std::memcpy(response.data, bytes, sizeof(bytes));
        return ::write(descriptor_, &response, CAN_MTU) == CAN_MTU;
    }

private:
    int descriptor_ = -1;
};

}  // namespace

int main()
{
    const char* configured = std::getenv("DAMIAO_CORE_VCAN_INTERFACE");
    if (configured == nullptr)
    {
        std::cout << "SKIP: set DAMIAO_CORE_VCAN_INTERFACE to an existing vcan interface\n";
        return 77;
    }
    const std::string interface(configured);
    // 此测试只接受明确指定的 vcan 名称，不自动选择或创建 can0。
    if (interface.rfind("vcan", 0) != 0 || interface.size() >= IFNAMSIZ)
    {
        std::cerr << "Only an explicitly configured vcan interface is allowed\n";
        return 1;
    }
    damiao::TransportConfig config;
    config.interface_name = interface;
    damiao::SocketCanTransport active;
    damiao::SocketCanTransport contender;
    damiao::SocketCanTransport passive;
    VirtualMotor motor;
    const auto opened = active.open(config);
    if (opened.code != damiao::ErrorCode::Ok)
    {
        std::cerr << opened.message << '\n';
        return 1;
    }
    if (contender.open(config).code != damiao::ErrorCode::OwnershipConflict)
    {
        return 1;
    }
    config.passive = true;
    if (passive.open(config).code != damiao::ErrorCode::Ok || !motor.open(interface))
    {
        return 1;
    }
    const auto request = damiao::DamiaoProtocol::encode_state_query(1).value.value();
    if (passive.send(request).code != damiao::ErrorCode::OwnershipConflict
        || active.send(request).code != damiao::ErrorCode::Ok || !motor.reply())
    {
        return 1;
    }
    const auto response = active.receive(damiao::SteadyClock::now() + std::chrono::milliseconds(100));
    if (!response.value || response.value->id != 0x11 || response.value->length != 8
        || response.value->data[6] != 30 || response.value->received_at > damiao::SteadyClock::now())
    {
        return 1;
    }
    if (active.receive(damiao::SteadyClock::now() + std::chrono::milliseconds(5)).status.code != damiao::ErrorCode::Timeout)
    {
        return 1;
    }
    active.close();
    config.passive = false;
    if (contender.open(config).code != damiao::ErrorCode::Ok)
    {
        return 1;
    }
    std::cout << "PASS: vcan kernel receive, passive send gate, timeout and ownership\n";
    return 0;
}
