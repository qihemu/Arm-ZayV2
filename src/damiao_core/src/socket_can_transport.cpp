#include <damiao_core/transport.hpp>
#include "socket_can_detail.hpp"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstring>
#include <fcntl.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

namespace damiao
{
namespace
{

// 将常见 Linux I/O errno 归类为稳定的核心错误码并保留诊断文本。
Status system_error(const char* operation, int error)
{
    ErrorCode code = ErrorCode::BusError;
    if (error == EAGAIN || error == EWOULDBLOCK || error == ENOBUFS || error == EINTR)
    {
        code = ErrorCode::WouldBlock;
    }
    else if (error == ENODEV || error == ENETDOWN || error == EBADF || error == ENXIO)
    {
        code = ErrorCode::Disconnected;
    }
    return {code, std::string(operation) + ": " + std::strerror(error)};
}

}  // namespace

namespace detail
{

// 校验原生帧类别和尺寸后复制到固定容量核心帧。
Result<CanFrame> decode_native_frame(const can_frame& native, std::size_t bytes,
    SteadyClock::time_point received_at)
{
    if (bytes != CAN_MTU)
    {
        return {{ErrorCode::InvalidFrame, "Not a complete classical CAN frame."}, std::nullopt};
    }
    if ((native.can_id & CAN_ERR_FLAG) != 0)
    {
        return {{ErrorCode::BusError, "CAN error flags: " + std::to_string(native.can_id & CAN_ERR_MASK)},
            std::nullopt};
    }
    if ((native.can_id & ~CAN_SFF_MASK) != 0 || native.len > CAN_MAX_DLEN)
    {
        return {{ErrorCode::InvalidFrame, "Extended/RTR/invalid CAN frame rejected."}, std::nullopt};
    }
    CanFrame frame;
    frame.id = static_cast<std::uint16_t>(native.can_id);
    frame.length = native.len;
    std::copy_n(native.data, frame.length, frame.data.begin());
    frame.received_at = received_at;
    return {{}, frame};
}

// 校验核心帧范围后复制到 Linux 经典 CAN 帧。
Result<can_frame> encode_native_frame(const CanFrame& frame)
{
    if (frame.id > CAN_SFF_MASK || frame.length > CAN_MAX_DLEN)
    {
        return {{ErrorCode::InvalidFrame, "Invalid classical CAN ID or payload length."}, std::nullopt};
    }
    can_frame native{};
    native.can_id = frame.id;
    native.len = frame.length;
    std::copy_n(frame.data.begin(), frame.length, native.data);
    return {{}, native};
}

// flock 对不同实例也互斥；锁文件不删除，避免旧 inode 与新文件同时被持有。
Status acquire_lock(const std::string& path, int& descriptor)
{
    if (descriptor != -1)
    {
        return {ErrorCode::InvalidConfiguration, "Ownership already acquired."};
    }
    int candidate = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0666);
    if (candidate < 0)
    {
        return system_error("open ownership lock", errno);
    }
    struct stat metadata{};
    if (::fstat(candidate, &metadata) != 0 || !S_ISREG(metadata.st_mode))
    {
        ::close(candidate);
        return {ErrorCode::OwnershipConflict, "Ownership lock is not a regular file."};
    }
    if (::flock(candidate, LOCK_EX | LOCK_NB) != 0)
    {
        const int error = errno;
        char owner[128]{};
        const auto count = ::pread(candidate, owner, sizeof(owner) - 1, 0);
        ::close(candidate);
        if (error == EWOULDBLOCK || error == EAGAIN)
        {
            return {ErrorCode::OwnershipConflict, "CAN interface owned by "
                + std::string(owner, count > 0 ? static_cast<std::size_t>(count) : 0)};
        }
        return system_error("flock", error);
    }
    const auto owner = "pid=" + std::to_string(::getpid()) + "\n";
    if (::ftruncate(candidate, 0) != 0
        || ::pwrite(candidate, owner.data(), owner.size(), 0) != static_cast<ssize_t>(owner.size()))
    {
        const int error = errno;
        ::close(candidate);
        return system_error("record ownership", error);
    }
    descriptor = candidate;
    return {};
}

// 关闭所有权锁并复位描述符，使重复释放保持安全。
void release_lock(int& descriptor) noexcept
{
    if (descriptor >= 0)
    {
        ::close(descriptor);
        descriptor = -1;
    }
}

}  // namespace detail

// 初始描述符均为无效值，构造过程不访问 CAN 设备。
SocketCanTransport::SocketCanTransport() = default;

SocketCanTransport::~SocketCanTransport()
{
    // 析构只回收 Socket 和协作锁，不发送管理或停车帧。
    close();
}

Status SocketCanTransport::open(const TransportConfig& config)
{
    if (is_open() || config.interface_name.empty() || config.interface_name.size() >= IFNAMSIZ
        || config.interface_name.find('\0') != std::string::npos || config.lock_directory.empty())
    {
        return {ErrorCode::InvalidConfiguration, "Invalid interface name/lock directory or already open."};
    }
    socket_fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (socket_fd_ < 0)
    {
        return system_error("socket", errno);
    }
    // 失败路径始终关闭部分资源；连接不会配置波特率，也不会自动重试。
    const auto fail = [this](const char* operation)
    {
        const auto status = system_error(operation, errno);
        close();
        return status;
    };
    struct ifreq request{};
    std::memcpy(request.ifr_name, config.interface_name.c_str(), config.interface_name.size() + 1);
    if (::ioctl(socket_fd_, SIOCGIFINDEX, &request) != 0)
    {
        return fail("SIOCGIFINDEX");
    }
    const int interface_index = request.ifr_ifindex;
    if (::ioctl(socket_fd_, SIOCGIFFLAGS, &request) != 0)
    {
        return fail("SIOCGIFFLAGS");
    }
    if ((request.ifr_flags & IFF_UP) == 0)
    {
        close();
        return {ErrorCode::Disconnected, "CAN interface is down."};
    }
    if (::ioctl(socket_fd_, SIOCGIFMTU, &request) != 0)
    {
        return fail("SIOCGIFMTU");
    }
    if (request.ifr_mtu != CAN_MTU && request.ifr_mtu != CANFD_MTU)
    {
        close();
        return {ErrorCode::InvalidConfiguration, "Interface is not CAN."};
    }
    passive_ = config.passive;
    if (!passive_)
    {
        struct stat identity{};
        if (::stat("/proc/self/ns/net", &identity) != 0)
        {
            return fail("stat network namespace");
        }
        // 命名使用网络命名空间与 ifindex，避免同一接口别名获得不同锁。
        const auto path = config.lock_directory + "/damiao-core-"
            + std::to_string(identity.st_dev) + "-" + std::to_string(identity.st_ino)
            + "-" + std::to_string(interface_index) + ".lock";
        const auto status = detail::acquire_lock(path, lock_fd_);
        if (status.code != ErrorCode::Ok)
        {
            close();
            return status;
        }
    }
    const int off = 0;
    const int on = 1;
    const can_err_mask_t errors = CAN_ERR_MASK;
    const can_filter standard_filter{0, CAN_EFF_FLAG | CAN_RTR_FLAG};
    // 内核过滤扩展帧和 RTR，保留错误帧用于总线故障诊断，并请求接收时间戳。
    if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &off, sizeof(off)) != 0
        || ::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &off, sizeof(off)) != 0
        || ::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &standard_filter, sizeof(standard_filter)) != 0
        || ::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) != 0
        || ::setsockopt(socket_fd_, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof(on)) != 0)
    {
        return fail("setsockopt");
    }
    struct sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = interface_index;
    if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&address), sizeof(address)) != 0)
    {
        return fail("bind");
    }
    return {};
}

Status SocketCanTransport::send(const CanFrame& frame)
{
    const auto code = send_code(frame);
    return code == ErrorCode::Ok ? Status{} : Status{code, "SocketCAN frame send rejected or failed."};
}

ErrorCode SocketCanTransport::send_code(const CanFrame& frame)
{
    if (!is_open())
    {
        return ErrorCode::Disconnected;
    }
    if (passive_)
    {
        return ErrorCode::OwnershipConflict;
    }
    if (frame.id > CAN_SFF_MASK || frame.length > CAN_MAX_DLEN)
    {
        return ErrorCode::InvalidFrame;
    }
    // 周期发送直接使用栈上定长帧，不分配、不排队、不构造错误字符串。
    can_frame native{};
    native.can_id = frame.id;
    native.len = frame.length;
    std::copy_n(frame.data.begin(), frame.length, native.data);
    const ssize_t bytes = ::send(socket_fd_, &native, CAN_MTU, MSG_DONTWAIT | MSG_NOSIGNAL);
    if (bytes < 0)
    {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS)
        {
            return ErrorCode::WouldBlock;
        }
        if (errno == ENODEV || errno == ENETDOWN || errno == EBADF || errno == ENXIO)
        {
            return ErrorCode::Disconnected;
        }
        return ErrorCode::BusError;
    }
    return bytes == CAN_MTU ? ErrorCode::Ok : ErrorCode::BusError;
}

Result<CanFrame> SocketCanTransport::receive(Deadline deadline)
{
    if (!is_open())
    {
        return {{ErrorCode::Disconnected, "CAN socket is closed."}, std::nullopt};
    }
    while (SteadyClock::now() < deadline)
    {
        // 每轮按剩余绝对期限计算 poll 超时，EINTR 后也不会延长调用期限。
        const auto remaining = deadline - SteadyClock::now();
        if (remaining <= SteadyClock::duration::zero())
        {
            break;
        }
        const auto milliseconds = std::chrono::ceil<std::chrono::milliseconds>(remaining).count();
        struct pollfd descriptor{socket_fd_, POLLIN, 0};
        const int ready = ::poll(&descriptor, 1, static_cast<int>(std::min<long long>(milliseconds, INT_MAX)));
        if (ready < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            return {system_error("poll", errno), std::nullopt};
        }
        if (ready == 0)
        {
            continue;
        }
        if ((descriptor.revents & (POLLHUP | POLLNVAL)) != 0)
        {
            return {{ErrorCode::Disconnected, "CAN socket disconnected."}, std::nullopt};
        }
        if ((descriptor.revents & POLLIN) == 0)
        {
            return {{ErrorCode::BusError, "CAN socket poll error."}, std::nullopt};
        }
        can_frame native{};
        struct iovec buffer{&native, sizeof(native)};
        alignas(struct cmsghdr) char control[CMSG_SPACE(sizeof(struct timespec))]{};
        struct msghdr message{};
        message.msg_iov = &buffer;
        message.msg_iovlen = 1;
        message.msg_control = control;
        message.msg_controllen = sizeof(control);
        const ssize_t bytes = ::recvmsg(socket_fd_, &message, MSG_DONTWAIT | MSG_TRUNC);
        if (bytes < 0)
        {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
            {
                continue;
            }
            return {system_error("recvmsg", errno), std::nullopt};
        }
        if (bytes == 0)
        {
            return {{ErrorCode::Disconnected, "CAN socket closed during receive."}, std::nullopt};
        }
        if ((message.msg_flags & MSG_CTRUNC) != 0)
        {
            return {{ErrorCode::InvalidFrame, "Truncated CAN timestamp metadata."}, std::nullopt};
        }
        auto received_at = SteadyClock::now();
        bool timestamp_found = false;
        // 内核时间戳为 realtime；按当前两时钟差估算 steady 接收时刻，保留队列年龄。
        // 系统时钟跳变会影响该估算，不能把它当作设备采样时刻。
        for (auto* header = CMSG_FIRSTHDR(&message); header != nullptr; header = CMSG_NXTHDR(&message, header))
        {
            if (header->cmsg_level == SOL_SOCKET && header->cmsg_type == SCM_TIMESTAMPNS
                && header->cmsg_len >= CMSG_LEN(sizeof(struct timespec)))
            {
                struct timespec stamp{};
                std::memcpy(&stamp, CMSG_DATA(header), sizeof(stamp));
                const auto kernel_time = std::chrono::seconds(stamp.tv_sec) + std::chrono::nanoseconds(stamp.tv_nsec);
                const auto realtime_now = std::chrono::system_clock::now().time_since_epoch();
                const auto steady_now = SteadyClock::now();
                const auto age = realtime_now - kernel_time;
                if (age < -std::chrono::milliseconds(1))
                {
                    return {{ErrorCode::InvalidFrame, "Kernel timestamp is in the future; realtime clock changed."}, std::nullopt};
                }
                timestamp_found = true;
                received_at = steady_now;
                if (age > decltype(age)::zero())
                {
                    received_at -= std::chrono::duration_cast<SteadyClock::duration>(age);
                }
            }
        }
        if (!timestamp_found)
        {
            return {{ErrorCode::InvalidFrame, "CAN receive timestamp is missing."}, std::nullopt};
        }
        return detail::decode_native_frame(native, static_cast<std::size_t>(bytes), received_at);
    }
    return {{ErrorCode::Timeout, "CAN receive deadline expired."}, std::nullopt};
}

Status SocketCanTransport::close()
{
    Status status;
    if (socket_fd_ >= 0)
    {
        const int descriptor = socket_fd_;
        socket_fd_ = -1;
        if (::close(descriptor) != 0)
        {
            status = system_error("close", errno);
        }
    }
    detail::release_lock(lock_fd_);
    passive_ = false;
    return status;
}

bool SocketCanTransport::is_open() const noexcept
{
    return socket_fd_ >= 0;
}

}  // namespace damiao
