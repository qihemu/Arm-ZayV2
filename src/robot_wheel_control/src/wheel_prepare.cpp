#include <robot_wheel_control/configuration.hpp>
#include <damiao_core/h55/bus.hpp>
#include <damiao_core/transports/dm_usb_sdk_transport.hpp>
#include <atomic>
#include <iostream>
#include <thread>

// Same disabled startup verification as the runtime; never calls enable or sends velocity.
int main(int argc, char **argv)
{
    try
    {
        if (argc != 2)
        {
            throw std::runtime_error("Usage: wheel_prepare /absolute/config.yaml");
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
        const auto prepared = bus.verify_configuration();
        if (prepared.code != damiao::ErrorCode::Ok)
        {
            throw std::runtime_error(prepared.message);
        }
        std::cout << "Protection initialized and read back; both drivers disabled; no Flash save. digest="
                  << c.digest << std::endl;

    }
    catch (const std::exception &e)
    {
        std::cerr << "Prepare failed: " << e.what() << std::endl;
        return 1;
    }
}
