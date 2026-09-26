#include <robot_wheel_control/configuration.hpp>
#include <damiao_core/h55/bus.hpp>
#include <damiao_core/transports/dm_usb_sdk_transport.hpp>
#include <atomic>
#include <iostream>
#include <thread>

// 只读工具：使用正式C++传输适配与H55解码；不调用verify_configuration（包含FD）、
// enable、速度命令或写寄存器。单次1秒读参期限仅属于此诊断程序，不放宽运动线程期限。
int main(int argc, char **argv)
{
    try
    {
        if (argc != 2)
        {
            throw std::runtime_error("Usage: wheel_readback /absolute/config.yaml");
        }
        auto c = robot_wheel_control::load_configuration(argv[1]);
        c.bus.reply_timeout = std::chrono::milliseconds(1000);
        damiao::h55::H55Bus bus(c.bus, std::make_unique<damiao::DmUsbCanTransport>(c.sdk));
        const auto opened = bus.open(c.transport);
        if (opened.code != damiao::ErrorCode::Ok)
        {
            throw std::runtime_error(opened.message);
        }
        struct Receiver
        {
            damiao::h55::H55Bus &bus;
            std::atomic<bool> running{true};
            std::thread thread;
            explicit Receiver(damiao::h55::H55Bus &b)
                : bus(b),
                  thread(
                      [this]
                      {
                          while (running)
                          {
                              bus.receive_once(damiao::SteadyClock::now() + std::chrono::milliseconds(5));
                          }
                      })
            {
            }
            ~Receiver()
            {
                running = false;
                thread.join();
                bus.close();
            }
        } receiver(bus);
        for (std::size_t wheel = 0; wheel < 2; ++wheel)
        {
            for (auto rid : {8, 7, 10, 14, 0x24, 0x15, 0x16, 0x17, 0x23, 9, 6, 0x3c, 0x3d, 0x3e})
            {
                const auto start = damiao::SteadyClock::now();
                const auto value = bus.read_register(wheel, rid);
                if (!value.value)
                {
                    throw std::runtime_error(value.status.message);
                }
                std::cout << "wheel=" << wheel << " rid=" << rid << " value=";
                std::visit([](auto v) { std::cout << v; }, *value.value);
                std::cout
                    << " round_trip_ms="
                    << std::chrono::duration<double, std::milli>(damiao::SteadyClock::now() - start).count()
                    << std::endl;
            }
        }
        std::cout << "Completed 28 parameter reads; no motor writes or motion commands." << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Readback failed: " << e.what() << std::endl;
        return 1;
    }
}
