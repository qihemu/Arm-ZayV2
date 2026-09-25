#include <damiao_core/transports/dm_usb_sdk_transport.hpp>
#include <atomic>
#include <algorithm>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <dlfcn.h>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <cmath>
#include <stdexcept>

namespace damiao
{
namespace
{
// [共享传输] ABI依据厂商DM_DeviceSDK/C&C++/lib/v1.1.0/dmcan.h，避免位域布局依赖。
#pragma pack(push, 1)
struct SdkFrame
{
    std::uint32_t id_flags;
    std::uint64_t timestamp;
    std::uint8_t channel;
    std::uint8_t flags_dlc;
    std::uint16_t reserved;
    std::uint8_t payload[64];
};
struct CanInfo
{
    std::uint8_t channel;
    bool canfd;
    std::uint32_t bitrate;
    std::uint32_t fd_bitrate;
    float sample;
    float fd_sample;
};
#pragma pack(pop)
static_assert(sizeof(SdkFrame) == 80 && sizeof(CanInfo) == 18);
struct Inbox
{
    std::mutex mutex;
    std::condition_variable changed;
    std::deque<CanFrame> frames;
    std::size_t capacity = 128;
    std::uint8_t channel = 0;
    std::atomic<bool> failed{false};
};
// SDK回调无用户上下文，仅按设备句柄路由到独立收件箱，不共享电机/控制状态。
std::mutex registry_mutex;
std::unordered_map<void *, std::shared_ptr<Inbox>> inboxes;
void received(void *handle, SdkFrame *f)
{
    std::shared_ptr<Inbox> box;
    {
        std::lock_guard<std::mutex> lock(registry_mutex);
        const auto it = inboxes.find(handle);
        if (it == inboxes.end())
        {
            return;
        }
        box = it->second;
    }
    if (!f || f->channel != box->channel || (f->id_flags & 0xe0000000U) ||
        (f->id_flags & 0x1fffffffU) > 0x7ff || (f->flags_dlc & 0x07) || (f->flags_dlc >> 4) > 8)
    {
        return;
    }
    CanFrame frame;
    frame.id = f->id_flags;
    frame.length = f->flags_dlc >> 4;
    frame.received_at = SteadyClock::now();
    std::copy_n(f->payload, frame.length, frame.data.begin());
    std::lock_guard<std::mutex> lock(box->mutex);
    if (box->frames.size() >= box->capacity)
    {
        box->failed = true;
    }
    else
    {
        box->frames.push_back(frame);
    }
    box->changed.notify_all();
}
std::string read_text(const std::filesystem::path &p)
{
    std::ifstream file(p);
    std::string text;
    std::getline(file, text);
    return text;
}
} // namespace
class DmUsbCanTransport::Impl
{
  public:
    explicit Impl(DmUsbSdkConfig c) : config(std::move(c))
    {
    }
    DmUsbSdkConfig config;
    void *library = nullptr;
    void *context = nullptr;
    void *device = nullptr;
    int lock_fd = -1;
    std::atomic<bool> opened{false};
    std::atomic<bool> running{false};
    std::shared_ptr<Inbox> inbox = std::make_shared<Inbox>();
    std::timed_mutex tx_mutex;
    std::condition_variable_any tx_changed;
    std::deque<std::pair<CanFrame, Deadline>> tx;
    std::thread sender;
    void (*create)(void **) = nullptr;
    void (*destroy)(void *) = nullptr;
    int (*find)(void *) = nullptr;
    bool (*get)(void *, void **, int) = nullptr;
    bool (*device_open)(void *) = nullptr;
    bool (*get_baud)(void *, std::uint8_t, CanInfo *) = nullptr;
    void (*hook)(void *, void (*)(void *, SdkFrame *)) = nullptr;
    bool (*enable)(void *, std::uint8_t) = nullptr;
    bool (*send_can)(void *, std::uint8_t, std::uint32_t, bool, bool, bool, bool, std::uint8_t,
                     std::uint8_t *) = nullptr;
    template <class T> void symbol(T &target, const char *name)
    {
        target = reinterpret_cast<T>(dlsym(library, name));
        if (!target)
        {
            throw std::runtime_error(std::string("Missing SDK symbol ") + name);
        }
    }
    void send_loop()
    {
        while (running)
        {
            std::unique_lock<std::timed_mutex> lock(tx_mutex);
            tx_changed.wait(lock, [this] { return !running || !tx.empty(); });
            if (!running)
            {
                break;
            }
            auto item = tx.front();
            tx.pop_front();
            lock.unlock();
            const auto &queued = item.first;
            const bool disable =
                queued.length == 8 && queued.data[7] == 0xfd &&
                std::all_of(queued.data.begin(), queued.data.begin() + 7, [](auto b) { return b == 0xff; });
            // 异步错误已锁存时，先前排队的运动目标也必须丢弃；仅保留显式失能尝试。
            if (inbox->failed && !disable)
            {
                continue;
            }
            // 旧队列报文在进入SDK前过期丢弃并锁存错误；不伪报执行成功。
            if (SteadyClock::now() > item.second)
            {
                inbox->failed = true;
                continue;
            }
            auto &f = item.first;
            const auto before = SteadyClock::now();
            const bool ok =
                send_can(device, config.channel, f.id, false, false, false, false, f.length, f.data.data());
            if (!ok || SteadyClock::now() - before > config.send_deadline)
            {
                inbox->failed = true;
                inbox->changed.notify_all();
            }
        }
    }
};
DmUsbCanTransport::DmUsbCanTransport(DmUsbSdkConfig config) : impl_(std::make_unique<Impl>(std::move(config)))
{
}
DmUsbCanTransport::~DmUsbCanTransport()
{
    close();
}
Status DmUsbCanTransport::open(const TransportConfig &config)
{
    auto &x = *impl_;
    if (x.opened)
    {
        return {ErrorCode::InvalidConfiguration, "SDK already open"};
    }
    if (config.passive || x.config.library_path.empty() || x.config.serial_number.empty() ||
        x.config.bitrate != 1000000 || x.config.channel != 0 || x.config.receive_capacity < 2)
    {
        return {ErrorCode::InvalidConfiguration, "SDK requires explicit serial/path, channel0 Classic1M"};
    }
    try
    {
        // v1.1.0无序列号API：先sysfs核验唯一34b7:6877适配器，再SDK唯一设备计数。
        int count = 0;
        bool serial_matches = false;
        for (const auto &entry : std::filesystem::directory_iterator("/sys/bus/usb/devices"))
        {
            if (read_text(entry.path() / "idVendor") == "34b7" &&
                read_text(entry.path() / "idProduct") == "6877")
            {
                ++count;
                serial_matches = read_text(entry.path() / "serial") == x.config.serial_number;
            }
        }
        if (count != 1 || !serial_matches)
        {
            throw std::runtime_error("Exactly one USB adapter with matching sysfs serial required");
        }
        std::filesystem::create_directories(config.lock_directory);
        // 单SDK适配器模式使用固定协作锁，不能靠改变配置字符串绕过。
        const auto path = std::filesystem::path(config.lock_directory) / "damiao-usb-sdk-channel0.lock";
        x.lock_fd = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (x.lock_fd < 0 || flock(x.lock_fd, LOCK_EX | LOCK_NB))
        {
            throw std::runtime_error("USB SDK ownership conflict");
        }
        x.library = dlopen(x.config.library_path.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (!x.library)
        {
            throw std::runtime_error(dlerror());
        }
        x.symbol(x.create, "dmcan_context_create");
        x.symbol(x.destroy, "dmcan_context_destroy");
        x.symbol(x.find, "dmcan_find_devices");
        x.symbol(x.get, "dmcan_device_get");
        x.symbol(x.device_open, "dmcan_device_open");
        x.symbol(x.get_baud, "dmcan_device_get_channel_baudrate");
        x.symbol(x.hook, "dmcan_device_hook_recv_callback");
        x.symbol(x.enable, "dmcan_device_enable_channel");
        x.symbol(x.send_can, "dmcan_device_send_can");
        x.create(&x.context);
        if (!x.context || x.find(x.context) != 1 || !x.get(x.context, &x.device, 0) ||
            !x.device_open(x.device))
        {
            throw std::runtime_error("SDK open/count failed");
        }
        CanInfo info{};
        if (!x.get_baud(x.device, x.config.channel, &info) || info.canfd ||
            info.bitrate != x.config.bitrate || std::abs(info.sample - x.config.sample_point) > 1e-4)
        {
            throw std::runtime_error("SDK adapter CAN baud/sample point mismatch; not changed");
        }
        x.inbox = std::make_shared<Inbox>();
        x.inbox->capacity = x.config.receive_capacity;
        x.inbox->channel = x.config.channel;
        {
            std::lock_guard<std::mutex> lock(registry_mutex);
            inboxes[x.device] = x.inbox;
        }
        x.hook(x.device, received);
        if (!x.enable(x.device, x.config.channel))
        {
            throw std::runtime_error("SDK channel enable failed");
        }
        x.running = true;
        x.opened = true;
        x.sender = std::thread([&x] { x.send_loop(); });
        return {};
    }
    catch (const std::exception &e)
    {
        close();
        return {ErrorCode::Disconnected, e.what()};
    }
}
Status DmUsbCanTransport::close()
{
    auto &x = *impl_;
    x.opened = false;
    x.running = false;
    x.tx_changed.notify_all();
    if (x.sender.joinable())
    {
        x.sender.join();
    }
    {
        std::lock_guard<std::mutex> lock(registry_mutex);
        inboxes.erase(x.device);
    }
    if (x.context && x.destroy)
    {
        x.destroy(x.context);
    }
    x.context = nullptr;
    x.device = nullptr;
    if (x.library)
    {
        dlclose(x.library);
    }
    x.library = nullptr;
    if (x.lock_fd >= 0)
    {
        ::close(x.lock_fd);
    }
    x.lock_fd = -1;
    x.tx.clear();
    return {};
}
Status DmUsbCanTransport::send(const CanFrame &frame)
{
    auto &x = *impl_;
    if (!x.opened)
    {
        return {ErrorCode::Disconnected, "SDK closed"};
    }
    if (frame.id > 0x7ff || frame.length > 8)
    {
        return {ErrorCode::InvalidFrame, "Invalid Classic frame"};
    }
    const auto deadline = SteadyClock::now() + x.config.send_deadline;
    // SDK调用在队列锁之外。发送线程短暂取队列是正常竞争，不能一次try_lock失败
    // 就把双轮锁成故障；允许最多2ms获取短临界区，真正超时仍明确失败。
    std::unique_lock<std::timed_mutex> lock(x.tx_mutex, std::defer_lock);
    if (!lock.try_lock_until(std::min(deadline, SteadyClock::now() + std::chrono::milliseconds(2))))
    {
        return {ErrorCode::WouldBlock, "SDK TX queue lock deadline exceeded (2ms)"};
    }
    // 仅识别专用停止帧以清除在途队列；不自动生成电机控制动作。
    const bool disable =
        frame.length == 8 && frame.data[7] == 0xfd &&
        std::all_of(frame.data.begin(), frame.data.begin() + 7, [](auto b) { return b == 0xff; });
    if (disable)
    {
        x.tx.erase(std::remove_if(x.tx.begin(), x.tx.end(),
                                  [&frame](const auto &item) { return item.first.id == frame.id; }),
                   x.tx.end());
    }
    if (x.inbox->failed && !disable)
    {
        return {ErrorCode::BusError, "SDK asynchronous send/RX failure"};
    }
    if (x.tx.size() >= 16)
    {
        return {ErrorCode::WouldBlock, "SDK TX full"};
    }
    const auto item = std::make_pair(frame, deadline);
    if (disable)
    {
        x.tx.push_front(item);
    }
    else
    {
        x.tx.push_back(item);
    }
    x.tx_changed.notify_one();
    return {}; // 仅入队，不代表USB传输完成或电机执行。
}
Result<CanFrame> DmUsbCanTransport::receive(Deadline deadline)
{
    auto &x = *impl_;
    std::unique_lock<std::mutex> lock(x.inbox->mutex);
    x.inbox->changed.wait_until(lock, deadline,
                                [&x] { return x.inbox->failed || !x.inbox->frames.empty() || !x.opened; });
    if (x.inbox->failed)
    {
        return {{ErrorCode::BusError, "SDK asynchronous error or queue overflow"}, std::nullopt};
    }
    if (!x.opened)
    {
        return {{ErrorCode::Disconnected, "SDK closed"}, std::nullopt};
    }
    if (x.inbox->frames.empty())
    {
        return {{ErrorCode::Timeout, ""}, std::nullopt};
    }
    auto frame = x.inbox->frames.front();
    x.inbox->frames.pop_front();
    return {{}, frame};
}
bool DmUsbCanTransport::is_open() const noexcept
{
    return impl_->opened;
}
} // namespace damiao
