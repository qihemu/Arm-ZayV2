#include <damiao_core/transport.hpp>

namespace damiao
{
namespace
{

// 当前后端不调用 Linux Socket API，明确报告功能尚未实现。
Status unsupported_transport_status()
{
    return {ErrorCode::Unsupported, "SocketCAN transport is not implemented yet."};
}

}  // namespace

SocketCanTransport::SocketCanTransport() = default;
SocketCanTransport::~SocketCanTransport() = default;

Status SocketCanTransport::open(const TransportConfig&)
{
    return unsupported_transport_status();
}

Status SocketCanTransport::send(const CanFrame&)
{
    return unsupported_transport_status();
}

Result<CanFrame> SocketCanTransport::receive(Deadline)
{
    return {unsupported_transport_status(), std::nullopt};
}

// 无连接和资源，重复关闭始终安全，且不会隐式发送失能命令。
Status SocketCanTransport::close()
{
    return {};
}

bool SocketCanTransport::is_open() const noexcept
{
    return false;
}

}  // namespace damiao
