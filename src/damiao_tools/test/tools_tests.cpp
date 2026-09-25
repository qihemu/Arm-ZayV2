#include "action_sequence.hpp"
#include "config.hpp"
#include "motor_bus_session.hpp"
#include "motor_scanner.hpp"

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
#include <unordered_map>
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

CanFrame feedback(std::uint16_t mst_id, std::uint8_t esc_id, std::uint8_t status)
{
    CanFrame frame;
    frame.id = mst_id;
    frame.length = 8;
    frame.data = {static_cast<std::uint8_t>((status << 4) | esc_id), 0x80, 0x00, 0x80, 0x00, 0x00, 35, 40};
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

struct MotorSim
{
    std::uint16_t esc_id = 1;
    std::uint16_t mst_id = 0x11;
    std::uint8_t status = 0;
    std::uint32_t mode = 2;
    float pmax = 12.5F;
    float vmax = 30.0F;
    float tmax = 10.0F;
    std::uint32_t firmware = 72;
    std::vector<float> control_targets_;
    std::size_t zero_writes = 0;
};

// 模拟多台电机的管理响应和控制反馈，测试过程中不创建 CAN Socket。
class FakeTransport final : public ICanTransport
{
public:
    explicit FakeTransport(std::vector<MotorSim> motors) : motors_(std::move(motors))
    {
        for (auto& motor : motors_)
        {
            by_esc_[motor.esc_id] = &motor;
            by_mst_[motor.mst_id] = &motor;
        }
    }

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
        if (request.id == 0x7FF)
        {
            handle_management_transaction(request);
        }
        else if (request.length == 8 && request.data[7] >= 0xFB && request.data[7] <= 0xFE)
        {
            const auto esc = static_cast<std::uint16_t>(request.id & 0x0F);
            auto* motor = by_esc_.count(esc) ? by_esc_[esc] : nullptr;
            if (motor != nullptr)
            {
                if (request.data[7] == 0xFC)
                {
                    motor->status = 1;
                    ++enable_count_;
                }
                else if (request.data[7] == 0xFD)
                {
                    motor->status = 0;
                    ++disable_count_;
                }
                else if (request.data[7] == 0xFE)
                {
                    ++motor->zero_writes;
                }
            }
        }
        else if (request.length == 8)
        {
            if (fail_control_)
            {
                return {ErrorCode::BusError, "injected control send failure"};
            }
            const auto esc = static_cast<std::uint16_t>(request.id & 0x0F);
            auto* motor = by_esc_.count(esc) ? by_esc_[esc] : nullptr;
            if (motor != nullptr)
            {
                motor->control_targets_.push_back(load_float(request.data.data()));
                if (!drop_control_feedback_)
                {
                    queue_.push_back(feedback(motor->mst_id, motor->esc_id, motor->status));
                }
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

    void set_status(std::uint16_t esc, std::uint8_t status)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (by_esc_.count(esc))
        {
            by_esc_.at(esc)->status = status;
        }
    }

    bool saw_target(std::uint16_t esc, double target) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto* motor = by_esc_.count(esc) ? by_esc_.at(esc) : nullptr;
        if (motor == nullptr)
        {
            return false;
        }
        for (const auto value : motor->control_targets_)
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

    std::size_t save_count() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return save_count_;
    }

    std::size_t zero_count(std::uint16_t esc) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return by_esc_.count(esc) ? by_esc_.at(esc)->zero_writes : 0;
    }

private:
    void handle_management_transaction(const CanFrame& request)
    {
        const auto esc = static_cast<std::uint16_t>(request.data[0]);
        auto* motor = by_esc_.count(esc) ? by_esc_[esc] : nullptr;
        if (motor == nullptr)
        {
            return;
        }
        const auto operation = request.data[2];
        const auto rid = request.data[3];
        if (operation == 0xCC)
        {
            queue_.push_back(feedback(motor->mst_id, motor->esc_id, motor->status));
            return;
        }
        CanFrame response = request;
        response.id = motor->mst_id;
        response.length = 8;
        std::uint32_t bits = 0;
        if (operation == 0x33)
        {
            if (rid == 0x08)
            {
                bits = motor->esc_id;
            }
            else if (rid == 0x07)
            {
                bits = motor->mst_id;
            }
            else if (rid == 0x0A)
            {
                bits = motor->mode;
            }
            else if (rid == 0x0E)
            {
                bits = motor->firmware;
            }
            else if (rid == 0x15)
            {
                std::memcpy(&bits, &motor->pmax, sizeof(bits));
            }
            else if (rid == 0x16)
            {
                std::memcpy(&bits, &motor->vmax, sizeof(bits));
            }
            else if (rid == 0x17)
            {
                std::memcpy(&bits, &motor->tmax, sizeof(bits));
            }
            for (unsigned int byte = 0; byte < 4; ++byte)
            {
                response.data[4 + byte] = static_cast<std::uint8_t>(bits >> (8 * byte));
            }
        }
        else if (operation == 0x55 && rid == 0x0A)
        {
            bits = std::uint32_t(request.data[4]) | (std::uint32_t(request.data[5]) << 8)
                | (std::uint32_t(request.data[6]) << 16) | (std::uint32_t(request.data[7]) << 24);
            motor->mode = bits;
            for (unsigned int byte = 0; byte < 4; ++byte)
            {
                response.data[4 + byte] = static_cast<std::uint8_t>(bits >> (8 * byte));
            }
        }
        else if (operation == 0xAA)
        {
            response.length = 4;
            ++save_count_;
        }
        response.received_at = SteadyClock::now();
        queue_.push_back(response);
    }

    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<CanFrame> queue_;
    std::vector<CanFrame> sent_;
    std::vector<MotorSim> motors_;
    std::unordered_map<std::uint16_t, MotorSim*> by_esc_;
    std::unordered_map<std::uint16_t, MotorSim*> by_mst_;
    bool open_ = false;
    bool fail_control_ = false;
    bool drop_control_feedback_ = false;
    std::size_t enable_count_ = 0;
    std::size_t disable_count_ = 0;
    std::size_t save_count_ = 0;
};

ToolConfig tool_config()
{
    ToolConfig config;
    config.can_interface = "testcan0";
    config.min_output_position_rad = -1.0;
    config.max_output_position_rad = 1.0;
    config.max_output_speed_rad_s = 0.5;
    config.scan_esc_min = 1;
    config.scan_esc_max = 3;
    config.scan_timeout_ms = 200;
    return config;
}

void test_config()
{
    const std::string valid =
        "can_interface: can0\n"
        "min_output_position_rad: -1.0\n"
        "max_output_position_rad: 1.0\n"
        "max_output_speed_rad_s: 0.5\n";
    const auto parsed = parse_yaml(valid);
    check(parsed.status.code == ErrorCode::Ok && parsed.config.can_interface == "can0",
        "valid multi motor YAML");
    check(parse_yaml(valid + "unknown: 1\n").status.code == ErrorCode::InvalidConfiguration,
        "unknown YAML field rejected");
    check(parse_yaml("can_interface: can0\n").status.code == ErrorCode::InvalidConfiguration,
        "missing YAML fields rejected");
    check(parse_yaml(
        "can_interface: can0\nmin_output_position_rad: 1\nmax_output_position_rad: -1\n"
        "max_output_speed_rad_s: 1\n").status.code == ErrorCode::InvalidConfiguration,
        "reversed limits rejected");
    const auto with_sequence = parse_yaml(
        valid + "action_sequence_file: demo_sequence.txt\n");
    check(with_sequence.status.code == ErrorCode::Ok
        && with_sequence.config.action_sequence_file == "demo_sequence.txt",
        "optional action sequence field accepted");
}

std::string temporary_sequence(const std::string& content)
{
    char path[] = "/tmp/damiao-tools-sequence-XXXXXX";
    const int fd = ::mkstemp(path);
    check(fd >= 0, "create temporary sequence");
    ::close(fd);
    std::ofstream output(path);
    output << content;
    output.close();
    return path;
}

void test_action_sequence_parse()
{
    const auto path = temporary_sequence(
        "# comment\n"
        "\n"
        "M1 pos=0.5 ve=1.0\n"
        "delay 10\n"
        "M2 pos=-1.0 ve=0.2\n");
    const auto parsed = parse_action_sequence_file(path);
    ::unlink(path.c_str());
    check(parsed.status.code == ErrorCode::Ok, "sequence parse succeeds");
    check(parsed.steps.size() == 3, "sequence step count");
    check(parsed.steps[0].kind == ActionStepKind::Move && parsed.steps[0].motor_one_based == 1,
        "first move motor");
    check(parsed.steps[1].kind == ActionStepKind::Delay && parsed.steps[1].delay_ms == 10,
        "delay step");
    check(parsed.steps[2].motor_one_based == 2, "second move motor");

    const auto bad_path = temporary_sequence("M1 pos=1.0\n");
    const auto bad = parse_action_sequence_file(bad_path);
    ::unlink(bad_path.c_str());
    check(bad.status.code == ErrorCode::InvalidConfiguration, "invalid move line rejected");
}

void test_action_sequence_delay_does_not_wait_for_move()
{
    std::vector<ActionStep> steps;
    ActionStep first_move;
    first_move.kind = ActionStepKind::Move;
    first_move.motor_one_based = 1;
    first_move.position_rad = 1.0;
    first_move.speed_rad_s = 1.0;
    steps.push_back(first_move);

    ActionStep delay_step;
    delay_step.kind = ActionStepKind::Delay;
    delay_step.delay_ms = 40;
    steps.push_back(delay_step);

    ActionStep second_move;
    second_move.kind = ActionStepKind::Move;
    second_move.motor_one_based = 1;
    second_move.position_rad = 2.0;
    second_move.speed_rad_s = 1.0;
    steps.push_back(second_move);

    struct Event
    {
        char kind = 'M';
        double position = 0.0;
        SteadyClock::time_point at{};
    };
    std::vector<Event> events;
    const auto start = SteadyClock::now();
    std::atomic<bool> cancel{false};
    ActionSequenceCallbacks callbacks;
    callbacks.move = [&](std::size_t, double position_rad, double)
    {
        events.push_back({'M', position_rad, SteadyClock::now()});
        return Status{};
    };
    callbacks.wait = [&](std::uint32_t delay_ms, const std::atomic<bool>&)
    {
        events.push_back({'D', static_cast<double>(delay_ms), SteadyClock::now()});
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    };
    check(run_action_sequence(steps, callbacks, cancel).code == ErrorCode::Ok,
        "callback sequence run succeeds");
    check(events.size() == 3, "move delay move event count");
    check(events[0].kind == 'M' && events[1].kind == 'D' && events[2].kind == 'M',
        "event order");
    const auto gap_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        events[1].at - events[0].at).count();
    check(gap_ms < 5, "delay follows move without waiting for motion completion");
    const auto delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        events[2].at - events[1].at).count();
    check(delay_ms >= 35, "second move starts after delay elapses");
}

void test_resolve_sequence_path()
{
    check(resolve_sequence_path("/cfg", "demo.txt") == "/cfg/demo.txt", "relative path joined");
    check(resolve_sequence_path("/cfg", "/abs.seq") == "/abs.seq", "absolute path kept");
}

void test_scanner_finds_multiple_motors()
{
    std::vector<MotorSim> motors{
        {1, 0x11, 0, 2, 12.5F, 30.0F, 10.0F, 72},
        {2, 0x12, 0, 2, 12.5F, 30.0F, 10.0F, 73},
    };
    auto transport = std::make_unique<FakeTransport>(motors);
    const auto result = scan_motors(tool_config(), std::move(transport));
    check(result.status.code == ErrorCode::Ok, "scanner succeeds");
    check(result.motors.size() == 2, "scanner finds two motors");
    check(result.motors[0].esc_id == 1 && result.motors[1].esc_id == 2, "scanner esc ids");
    check(result.motors[0].operable && result.motors[1].operable, "scanner marks registered");
    check(result.motors[0].drivable && result.motors[1].drivable, "scanner marks pv drivable");
}

void test_bus_session_requires_enable_all_before_drive()
{
    std::vector<MotorSim> motors{{1, 0x11, 0, 2, 12.5F, 30.0F, 10.0F, 72}};
    DiscoveredMotor discovered;
    discovered.esc_id = 1;
    discovered.mst_id = 0x11;
    discovered.mode = ControlMode::PositionVelocity;
    discovered.pmax_rad = 12.5;
    discovered.vmax_rad_s = 30.0;
    discovered.tmax_nm = 10.0;
    discovered.firmware_version = 72;
    discovered.operable = true;
    discovered.drivable = true;

    auto transport = std::make_unique<FakeTransport>(motors);
    auto* fake = transport.get();
    MotorBusSession session(tool_config(), {discovered}, std::move(transport));
    check(session.initialize().code == ErrorCode::Ok, "session initializes");
    check(session.drive(0, 0.5, 0.2).code == ErrorCode::InvalidCommand, "drive before enable rejected");
    check(session.enable_all().code == ErrorCode::Ok && fake->enable_count() == 1, "enable all");
    check(session.motor_info(0).raw_status == 1, "enable updates displayed status");
    check(session.drive(0, 0.5, 0.2).code == ErrorCode::Ok, "drive after enable accepted");
    eventually([&]
    {
        return fake->saw_target(1, 0.5);
    }, "drive target reaches transport");
    check(session.disable_all().code == ErrorCode::Ok && fake->disable_count() == 1,
        "disable all");
    check(session.motor_info(0).raw_status == 0, "disable updates displayed status");
    session.shutdown();
}

void test_bus_session_holds_other_axes()
{
    std::vector<MotorSim> motors{
        {1, 0x11, 0, 2, 12.5F, 30.0F, 10.0F, 72},
        {2, 0x12, 0, 2, 12.5F, 30.0F, 10.0F, 73},
    };
    std::vector<DiscoveredMotor> discovered;
    for (const auto& sim : motors)
    {
        DiscoveredMotor motor;
        motor.esc_id = sim.esc_id;
        motor.mst_id = sim.mst_id;
        motor.mode = ControlMode::PositionVelocity;
        motor.pmax_rad = sim.pmax;
        motor.vmax_rad_s = sim.vmax;
        motor.tmax_nm = sim.tmax;
        motor.firmware_version = sim.firmware;
        motor.operable = true;
        motor.drivable = true;
        discovered.push_back(motor);
    }

    auto transport = std::make_unique<FakeTransport>(motors);
    auto* fake = transport.get();
    MotorBusSession session(tool_config(), discovered, std::move(transport));
    check(session.initialize().code == ErrorCode::Ok, "dual session initializes");
    check(session.enable_all().code == ErrorCode::Ok && fake->enable_count() == 2,
        "dual enable all");
    const double initial_m2 = (2.0 * (32768.0 / 65535.0) - 1.0) * 12.5;
    check(session.drive(0, 0.4, 0.2).code == ErrorCode::Ok, "drive axis 0");
    eventually([&]
    {
        return fake->saw_target(1, 0.4) && fake->saw_target(2, initial_m2);
    }, "non-driven axis keeps initial hold target");
    check(session.disable_all().code == ErrorCode::Ok, "dual disable all");
    session.shutdown();
}

void test_set_control_mode_requires_disable()
{
    std::vector<MotorSim> motors{{1, 0x11, 0, 2, 12.5F, 30.0F, 10.0F, 72}};
    DiscoveredMotor discovered;
    discovered.esc_id = 1;
    discovered.mst_id = 0x11;
    discovered.mode = ControlMode::PositionVelocity;
    discovered.pmax_rad = 12.5;
    discovered.vmax_rad_s = 30.0;
    discovered.tmax_nm = 10.0;
    discovered.firmware_version = 72;
    discovered.operable = true;
    discovered.drivable = true;

    MotorBusSession session(tool_config(), {discovered},
        std::make_unique<FakeTransport>(motors));
    check(session.initialize().code == ErrorCode::Ok, "mode session initializes");
    check(session.enable_all().code == ErrorCode::Ok, "mode session enables");
    check(session.set_control_mode(0, ControlMode::Mit).code == ErrorCode::InvalidCommand,
        "mode change rejected while enabled");
    check(session.disable_all().code == ErrorCode::Ok, "mode session disables");
    session.shutdown();
}

void test_set_control_mode_updates_metadata()
{
    std::vector<MotorSim> motors{{1, 0x11, 0, 2, 12.5F, 30.0F, 10.0F, 72}};
    DiscoveredMotor discovered;
    discovered.esc_id = 1;
    discovered.mst_id = 0x11;
    discovered.mode = ControlMode::PositionVelocity;
    discovered.pmax_rad = 12.5;
    discovered.vmax_rad_s = 30.0;
    discovered.tmax_nm = 10.0;
    discovered.firmware_version = 72;
    discovered.operable = true;
    discovered.drivable = true;

    MotorBusSession session(tool_config(), {discovered},
        std::make_unique<FakeTransport>(motors));
    check(session.initialize().code == ErrorCode::Ok, "mode metadata session initializes");
    check(session.set_control_mode(0, ControlMode::Mit).code == ErrorCode::Ok, "switch to MIT");
    const auto mit_mode = session.read_control_mode(0);
    check(mit_mode.value == ControlMode::Mit, "MIT mode readback");
    check(session.motor_info(0).mode == ControlMode::Mit, "session metadata updated");
    check(!session.motor_info(0).drivable, "MIT mode not drivable");
    check(session.enable_all().code == ErrorCode::InvalidCommand,
        "enable rejected for non position-velocity mode");
    check(session.set_control_mode(0, ControlMode::PositionVelocity).code == ErrorCode::Ok,
        "restore position-velocity mode");
    check(session.motor_info(0).mode == ControlMode::PositionVelocity, "PV mode restored");
    check(session.motor_info(0).drivable, "PV mode drivable again");
    check(session.enable_all().code == ErrorCode::Ok, "enable succeeds after PV restore");
    check(session.disable_all().code == ErrorCode::Ok, "mode metadata session disables");
    session.shutdown();
}

void test_non_pv_motor_registers_without_drive()
{
    std::vector<MotorSim> motors{{2, 0x12, 0, 1, 12.5F, 30.0F, 10.0F, 72}};
    DiscoveredMotor discovered;
    discovered.esc_id = 2;
    discovered.mst_id = 0x12;
    discovered.mode = ControlMode::Mit;
    discovered.pmax_rad = 12.5;
    discovered.vmax_rad_s = 30.0;
    discovered.tmax_nm = 10.0;
    discovered.firmware_version = 72;
    discovered.operable = true;
    discovered.drivable = false;
    discovered.inoperable_reason = "非位置速度模式";

    MotorBusSession session(tool_config(), {discovered},
        std::make_unique<FakeTransport>(motors));
    check(session.initialize().code == ErrorCode::Ok, "MIT motor registers");
    check(session.enable_all().code == ErrorCode::InvalidCommand, "MIT motor cannot enable all");
    check(session.drive(0, 0.1, 0.1).code == ErrorCode::InvalidCommand, "MIT motor cannot drive");
    session.shutdown();
}

void test_save_parameters_requires_disable()
{
    std::vector<MotorSim> motors{{1, 0x11, 0, 2, 12.5F, 30.0F, 10.0F, 72}};
    DiscoveredMotor discovered;
    discovered.esc_id = 1;
    discovered.mst_id = 0x11;
    discovered.mode = ControlMode::PositionVelocity;
    discovered.pmax_rad = 12.5;
    discovered.vmax_rad_s = 30.0;
    discovered.tmax_nm = 10.0;
    discovered.firmware_version = 72;
    discovered.operable = true;
    discovered.drivable = true;

    auto transport = std::make_unique<FakeTransport>(motors);
    auto* fake = transport.get();
    MotorBusSession session(tool_config(), {discovered}, std::move(transport));
    check(session.initialize().code == ErrorCode::Ok, "save session initializes");
    check(session.save_parameters(0).code == ErrorCode::Ok, "save after init with fresh query");
    check(session.bus_state() == BusState::Maintenance, "save keeps maintenance state");
    check(fake->save_count() == 1, "flash save after init");
    check(session.enable_all().code == ErrorCode::Ok, "save session enables");
    check(session.save_parameters(0).code == ErrorCode::InvalidCommand,
        "save rejected while enabled");
    check(session.disable_all().code == ErrorCode::Ok, "save session disables");
    check(session.save_parameters(0).code == ErrorCode::Ok, "save parameters succeeds");
    check(fake->save_count() == 2, "flash save after disable");
    session.shutdown();
}

void test_save_zero_targets_selected_motor_and_requires_disable()
{
    std::vector<MotorSim> motors{{1, 0x11, 0, 2, 12.5F, 30.0F, 10.0F, 72},
        {2, 0x12, 0, 2, 12.5F, 30.0F, 10.0F, 72}};
    std::vector<DiscoveredMotor> discovered;
    for (const auto& motor : motors)
    {
        DiscoveredMotor item;
        item.esc_id = motor.esc_id;
        item.mst_id = motor.mst_id;
        item.mode = ControlMode::PositionVelocity;
        item.pmax_rad = 12.5;
        item.vmax_rad_s = 30.0;
        item.tmax_nm = 10.0;
        item.firmware_version = 72;
        item.operable = true;
        item.drivable = true;
        discovered.push_back(item);
    }

    auto transport = std::make_unique<FakeTransport>(motors);
    auto* fake = transport.get();
    MotorBusSession session(tool_config(), discovered, std::move(transport));
    check(session.initialize().code == ErrorCode::Ok, "zero session initializes");
    // 仅对目标轴写零点，并验证使能期间不会发出零点命令。
    check(session.save_zero(1).code == ErrorCode::Ok, "zero write succeeds while disabled");
    check(fake->zero_count(1) == 0 && fake->zero_count(2) == 1,
        "zero write targets only the selected motor");
    check(std::abs(session.motor_info(1).output_position_rad) < 0.001,
        "zero write refreshes selected motor position");
    check(session.enable_all().code == ErrorCode::Ok, "zero session enables");
    check(session.save_zero(1).code == ErrorCode::InvalidCommand,
        "zero write rejected while enabled");
    check(fake->zero_count(2) == 1, "enabled rejection sends no zero command");
    check(session.disable_all().code == ErrorCode::Ok, "zero session disables");
    session.shutdown();

    // 会话未主动使能时，也必须拒绝总线上已有其他使能轴的零点写入。
    auto enabled_transport = std::make_unique<FakeTransport>(motors);
    auto* enabled_fake = enabled_transport.get();
    MotorBusSession external_session(tool_config(), discovered, std::move(enabled_transport));
    check(external_session.initialize().code == ErrorCode::Ok,
        "external-enabled session initializes");
    enabled_fake->set_status(1, 1);
    check(external_session.save_zero(1).code == ErrorCode::InvalidCommand,
        "zero write rejected when another motor is enabled");
    check(enabled_fake->zero_count(2) == 0,
        "other enabled motor blocks zero command transmission");
    external_session.shutdown();
}

}  // namespace

int main()
{
    try
    {
        test_config();
        test_action_sequence_parse();
        test_action_sequence_delay_does_not_wait_for_move();
        test_resolve_sequence_path();
        test_scanner_finds_multiple_motors();
        test_bus_session_requires_enable_all_before_drive();
        test_bus_session_holds_other_axes();
        test_set_control_mode_requires_disable();
        test_set_control_mode_updates_metadata();
        test_non_pv_motor_registers_without_drive();
        test_save_parameters_requires_disable();
        test_save_zero_targets_selected_motor_and_requires_disable();
        std::cout << "damiao_tools software tests passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "damiao_tools test failure: " << error.what() << '\n';
        return 1;
    }
}
