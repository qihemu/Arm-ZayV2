#include "config.hpp"
#include "motor_test_session.hpp"

#include <damiao_core/registers.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{

using namespace damiao;
using namespace damiao_tools;
using namespace std::chrono_literals;

void check(bool condition, const char* message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

template<typename Predicate>
void eventually(Predicate predicate, const char* message)
{
    const auto deadline = SteadyClock::now() + 2s;
    while (SteadyClock::now() < deadline)
    {
        if (predicate())
        {
            return;
        }
        std::this_thread::sleep_for(2ms);
    }
    throw std::runtime_error(message);
}

std::string temporary_yaml(const std::string& content)
{
    char path[] = "/tmp/damiao-tools-config-XXXXXX";
    const int fd = ::mkstemp(path);
    check(fd >= 0, "create temporary YAML");
    ::close(fd);
    std::ofstream output(path);
    output << content;
    output.close();
    return path;
}

ConfigResult parse_yaml(const std::string& content)
{
    const auto path = temporary_yaml(content);
    const auto result = load_config(path);
    ::unlink(path.c_str());
    return result;
}

void test_config()
{
    const std::string valid =
        "can_interface: can0\n"
        "esc_id: 1\n"
        "mst_id: 17\n"
        "min_output_position_rad: -1.0\n"
        "max_output_position_rad: 1.0\n"
        "max_output_speed_rad_s: 0.5\n";
    const auto parsed = parse_yaml(valid);
    check(parsed.status.code == ErrorCode::Ok && parsed.config.esc_id == 1
        && parsed.config.mst_id == 17, "valid single motor YAML");
    check(parse_yaml(valid + "unknown: 1\n").status.code == ErrorCode::InvalidConfiguration,
        "unknown YAML field rejected");
    check(parse_yaml("can_interface: can0\n").status.code == ErrorCode::InvalidConfiguration,
        "missing YAML fields rejected");
    check(parse_yaml(
        "can_interface: can0\nesc_id: 16\nmst_id: 17\nmin_output_position_rad: -1\n"
        "max_output_position_rad: 1\nmax_output_speed_rad_s: 1\n").status.code
        == ErrorCode::InvalidConfiguration, "invalid ESC rejected");
    check(parse_yaml(
        "can_interface: can0\nesc_id: 1\nmst_id: 17\nmin_output_position_rad: 1\n"
        "max_output_position_rad: -1\nmax_output_speed_rad_s: 1\n").status.code
        == ErrorCode::InvalidConfiguration, "reversed limits rejected");
    check(parse_yaml(
        "can_interface: can0\nesc_id: 1\nmst_id: 17\nmin_output_position_rad: -.inf\n"
        "max_output_position_rad: 1\nmax_output_speed_rad_s: 1\n").status.code
        == ErrorCode::InvalidConfiguration, "non-finite limit rejected");
    check(parse_yaml(
        "can_interface: can0\nesc_id: 1\nmst_id: 17\nmin_output_position_rad: -1\n"
        "max_output_position_rad: 1\nmax_output_speed_rad_s: 0\n").status.code
        == ErrorCode::InvalidConfiguration, "non-positive speed rejected");
}

CanFrame feedback(std::uint8_t status)
{
    CanFrame frame;
    frame.id = 0x11;
    frame.length = 8;
    frame.data = {static_cast<std::uint8_t>((status << 4) | 1), 0x80, 0x00, 0x80, 0x00, 0x00, 35, 40};
    frame.received_at = SteadyClock::now();
    return frame;
}

float load_float(const std::uint8_t* bytes)
{
    const std::uint32_t bits = std::uint32_t(bytes[0])
        | (std::uint32_t(bytes[1]) << 8)
        | (std::uint32_t(bytes[2]) << 16)
        | (std::uint32_t(bytes[3]) << 24);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

// 模拟单台电机的管理响应和控制反馈，测试过程中不创建 CAN Socket。
class FakeTransport final : public ICanTransport
{
public:
    Status open(const TransportConfig&) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        open_ = true;
        return {};
    }

    Status close() override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        open_ = false;
        ready_.notify_all();
        return {};
    }

    bool is_open() const noexcept override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return open_;
    }

    Status send(const CanFrame& request) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!open_)
        {
            return {ErrorCode::Disconnected, ""};
        }
        sent_.push_back(request);
        if (request.id == 0x7FF && request.data[0] == 1)
        {
            handle_management_transaction(request);
        }
        else if (request.id == 0x101 && request.length == 8
            && request.data[0] == 0xFF && request.data[7] >= 0xFB)
        {
            if (request.data[7] == 0xFC)
            {
                status_ = 1;
                ++enable_count_;
            }
            else if (request.data[7] == 0xFD)
            {
                status_ = 0;
                ++disable_count_;
            }
        }
        else if (request.id == 0x101 && request.length == 8)
        {
            if (fail_control_)
            {
                return {ErrorCode::BusError, "injected control send failure"};
            }
            control_targets_.push_back(load_float(request.data.data()));
            if (!drop_control_feedback_)
            {
                queue_.push_back(feedback(status_));
            }
        }
        ready_.notify_all();
        return {};
    }

    Result<CanFrame> receive(Deadline deadline) override
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!ready_.wait_until(lock, deadline, [this]
        {
            return !queue_.empty() || !open_;
        }))
        {
            return {{ErrorCode::Timeout, ""}, std::nullopt};
        }
        if (!open_)
        {
            return {{ErrorCode::Disconnected, ""}, std::nullopt};
        }
        auto frame = queue_.front();
        queue_.pop_front();
        return {{}, frame};
    }

    void fail_control(bool value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        fail_control_ = value;
    }

    void drop_control_feedback(bool value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        drop_control_feedback_ = value;
    }

    std::size_t control_count() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return control_targets_.size();
    }

    bool saw_target(double target) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto value : control_targets_)
        {
            if (std::abs(value - target) < 1e-5)
            {
                return true;
            }
        }
        return false;
    }

    std::size_t enable_count() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return enable_count_;
    }

    std::size_t disable_count() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return disable_count_;
    }

    std::uint8_t motor_status() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return status_;
    }

private:
    void handle_management_transaction(const CanFrame& request)
    {
        const auto operation = request.data[2];
        const auto rid = request.data[3];
        if (operation == 0xCC)
        {
            queue_.push_back(feedback(status_));
            return;
        }
        CanFrame response = request;
        response.id = 0x11;
        response.length = 8;
        std::uint32_t bits = 0;
        if (operation == 0x33)
        {
            if (rid == 0x08)
            {
                bits = 1;
            }
            else if (rid == 0x07)
            {
                bits = 0x11;
            }
            else if (rid == 0x0A)
            {
                bits = 2;
            }
            else if (rid == 0x0E)
            {
                bits = 72;
            }
            else
            {
                float value = rid == 0x15 ? 12.5F : (rid == 0x16 ? 30.0F : 10.0F);
                std::memcpy(&bits, &value, sizeof(bits));
            }
            for (unsigned int byte = 0; byte < 4; ++byte)
            {
                response.data[4 + byte] = static_cast<std::uint8_t>(bits >> (8 * byte));
            }
        }
        response.received_at = SteadyClock::now();
        queue_.push_back(response);
    }

    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<CanFrame> queue_;
    std::vector<CanFrame> sent_;
    std::vector<float> control_targets_;
    bool open_ = false;
    bool fail_control_ = false;
    bool drop_control_feedback_ = false;
    std::uint8_t status_ = 0;
    std::size_t enable_count_ = 0;
    std::size_t disable_count_ = 0;
};

ToolConfig tool_config()
{
    return {"testcan0", 1, 0x11, -1.0, 1.0, 0.5};
}

void test_session_control()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    MotorTestSession session(tool_config(), std::move(transport));
    check(session.initialize().code == ErrorCode::Ok, "session initializes");
    check(session.enable().code == ErrorCode::Ok && fake->enable_count() == 1, "motor enables explicitly");
    eventually([&]
    {
        return fake->control_count() > 1;
    }, "background hold sends repeatedly");
    const double initial_feedback_position = (2.0 * (32768.0 / 65535.0) - 1.0) * 12.5;
    check(fake->saw_target(initial_feedback_position), "first hold target uses measured position");
    check(session.drive(0.5, 0.2).code == ErrorCode::Ok, "valid absolute target accepted");
    eventually([&]
    {
        return fake->saw_target(0.5);
    }, "updated target reaches transport");
    check(session.drive(2.0, 0.2).code == ErrorCode::InvalidCommand, "out-of-range target rejected");
    check(!fake->saw_target(2.0), "rejected target never sent");
    check(session.disable().code == ErrorCode::Ok && fake->disable_count() == 1,
        "explicit disable confirmed");
    check(session.shutdown().code == ErrorCode::Ok, "disabled session closes");
}

void test_shutdown_does_not_disable()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    MotorTestSession session(tool_config(), std::move(transport));
    check(session.initialize().code == ErrorCode::Ok && session.enable().code == ErrorCode::Ok,
        "quit test enables");
    eventually([&]
    {
        return fake->control_count() > 0;
    }, "quit test control started");
    check(session.shutdown().code == ErrorCode::Ok, "enabled shutdown closes host resources");
    check(fake->disable_count() == 0 && fake->motor_status() == 1,
        "shutdown does not implicitly disable motor");
}

void test_failure_does_not_disable()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    MotorTestSession session(tool_config(), std::move(transport));
    check(session.initialize().code == ErrorCode::Ok && session.enable().code == ErrorCode::Ok,
        "failure test enables");
    fake->fail_control(true);
    eventually([&]
    {
        return session.background_error() == ErrorCode::BusError;
    }, "send failure stops background loop");
    check(fake->disable_count() == 0 && fake->motor_status() == 1,
        "send failure does not implicitly disable");
    check(session.disable().code == ErrorCode::Ok && fake->disable_count() == 1,
        "fault still permits explicit disable");
    session.shutdown();
}

void test_stale_feedback_does_not_disable()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    MotorTestSession session(tool_config(), std::move(transport));
    check(session.initialize().code == ErrorCode::Ok && session.enable().code == ErrorCode::Ok,
        "stale test enables");
    fake->drop_control_feedback(true);
    eventually([&]
    {
        return session.background_error() == ErrorCode::StaleFeedback;
    }, "stale feedback stops background loop");
    check(fake->disable_count() == 0 && fake->motor_status() == 1,
        "stale feedback does not implicitly disable");
    fake->drop_control_feedback(false);
    check(session.disable().code == ErrorCode::Ok, "stale fault explicit disable");
    session.shutdown();
}

}  // namespace

int main()
{
    try
    {
        test_config();
        test_session_control();
        test_shutdown_does_not_disable();
        test_failure_does_not_disable();
        test_stale_feedback_does_not_disable();
        std::cout << "damiao_tools software tests passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "damiao_tools test failure: " << error.what() << '\n';
        return 1;
    }
}
