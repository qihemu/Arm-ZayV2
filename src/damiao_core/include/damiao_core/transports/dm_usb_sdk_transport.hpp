#pragma once
#include <damiao_core/transport.hpp>
#include <memory>
namespace damiao
{
// [共享传输] SDK独立加载；调用方显式指定身份，不会打开第一个未核验设备。
struct DmUsbSdkConfig
{
    std::string library_path;
    std::string serial_number;
    std::uint8_t channel = 0;
    std::uint32_t bitrate = 1000000;
    float sample_point = 0.75F;
    std::size_t receive_capacity = 128;
    std::chrono::milliseconds send_deadline{20};
};
class DmUsbCanTransport final : public ICanTransport
{
  public:
    explicit DmUsbCanTransport(DmUsbSdkConfig config);
    ~DmUsbCanTransport() override;
    Status open(const TransportConfig &config) override;
    Status close() override;
    Status send(const CanFrame &frame) override;
    Result<CanFrame> receive(Deadline deadline) override;
    bool is_open() const noexcept override;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace damiao
