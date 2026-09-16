#include <damiao_core/bus.hpp>
#include <damiao_core/registers.hpp>
#include "socket_can_detail.hpp"

#include <cmath>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace
{

using namespace damiao;
using namespace std::chrono_literals;

void check(bool condition, const char* message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

bool near(double left, double right, double tolerance = 1e-9)
{
    return std::abs(left - right) <= tolerance;
}

Deadline deadline()
{
    return SteadyClock::now() + 1s;
}

// 独立构造反馈向量，避免只有库内部编码/解码往返测试。
CanFrame feedback(std::uint16_t esc, std::uint16_t mst, std::uint8_t status)
{
    CanFrame frame;
    frame.id = mst;
    frame.length = 8;
    frame.data = {static_cast<std::uint8_t>((status << 4) | esc), 0x80, 0x00, 0x80, 0x00, 0x00, 35, 40};
    frame.received_at = SteadyClock::now();
    return frame;
}

void test_protocol()
{
    const auto encoded = DamiaoProtocol::encode_position_velocity(1, {1.0, 2.0});
    check(encoded.value && encoded.value->id == 0x101 && encoded.value->length == 8, "position ID/DLC");
    check(encoded.value->data == std::array<std::uint8_t, 8>{0, 0, 0x80, 0x3F, 0, 0, 0, 0x40}, "float LE vector");
    check(DamiaoProtocol::encode_position_velocity(15, {-1.0, 0.0}).value->id == 0x10F, "upper ESC boundary");
    check(!DamiaoProtocol::encode_position_velocity(0, {0, 1}).value, "ESC zero rejected");
    check(!DamiaoProtocol::encode_position_velocity(16, {0, 1}).value, "ESC above 15 rejected");
    check(!DamiaoProtocol::encode_position_velocity(1, {0, -1}).value, "negative absolute speed rejected");
    check(!DamiaoProtocol::encode_position_velocity(1, {std::numeric_limits<double>::quiet_NaN(), 1}).value, "NaN rejected");
    check(!DamiaoProtocol::encode_position_velocity(1, {0, std::numeric_limits<double>::infinity()}).value, "Inf rejected");
    check(!DamiaoProtocol::encode_position_velocity(1, {std::numeric_limits<double>::max(), 1}).value, "float overflow rejected");
    check(!DamiaoProtocol::encode_position_velocity(1, {1e-300, 1}).value, "float underflow rejected");
    check(DamiaoProtocol::validate_address({1, 0x111}).code == ErrorCode::Ok, "full MST accepted");
    check(DamiaoProtocol::validate_address({1, 0x101}).code == ErrorCode::InvalidConfiguration, "low byte self conflict");
    check(DamiaoProtocol::validate_address({1, 0x7FF}).code == ErrorCode::InvalidConfiguration, "management ID conflict");
    check(DamiaoProtocol::validate_address({1, 0x800}).code == ErrorCode::InvalidConfiguration, "extended ID rejected");
    check(DamiaoProtocol::validate_mapping_limits({0, 1, 1}).code == ErrorCode::InvalidConfiguration, "zero mapping rejected");
    check(DamiaoProtocol::validate_mapping_limits({1, -1, 1}).code == ErrorCode::InvalidConfiguration, "negative mapping rejected");
    check(DamiaoProtocol::validate_mapping_limits({1, 1, std::numeric_limits<double>::infinity()}).code == ErrorCode::InvalidConfiguration, "nonfinite mapping rejected");

    auto frame = feedback(1, 0x111, 1);
    frame.data = {0x11, 0xFF, 0xFF, 0, 0x0F, 0xFF, 50, 60};
    const auto state = DamiaoProtocol::decode_feedback(frame, {1, 0x111}, {12.5, 30, 10}, 9);
    check(state.value && state.value->valid, "feedback valid");
    check(near(state.value->output_position_rad, 12.5) && near(state.value->output_velocity_rad_s, -30)
        && near(state.value->reported_torque_nm, 10), "independent endpoint feedback vector");
    check(state.value->raw_status == 1 && state.value->mos_temperature_c == 50
        && state.value->rotor_temperature_c == 60 && state.value->mapping_revision == 9
        && state.value->received_at == frame.received_at, "feedback metadata");
    const auto different_mapping = DamiaoProtocol::decode_feedback(frame, {1, 0x111}, {6, 10, 28});
    check(near(different_mapping.value->output_position_rad, 6) && near(state.value->output_position_rad, 12.5), "per motor mapping");
    check(!DamiaoProtocol::decode_feedback(frame, {1, 0x11}, {1, 1, 1}).value, "complete MST match");
    frame.data[0] = 0x12;
    check(!DamiaoProtocol::decode_feedback(frame, {1, 0x111}, {1, 1, 1}).value, "payload ESC match");
    frame.data[0] = 0xF1;
    check(DamiaoProtocol::decode_feedback(frame, {1, 0x111}, {1, 1, 1}).value->raw_status == 15, "unknown status preserved");
    check(std::string(DamiaoProtocol::status_description(15)) == "Unknown motor status", "unknown not normal");
    for (std::uint8_t status : {3, 4, 5, 8, 9, 10, 11, 12, 13, 14})
    {
        check(std::string(DamiaoProtocol::status_description(status)) != "Unknown motor status", "known fault dictionary");
    }
    frame.length = 7;
    check(!DamiaoProtocol::decode_feedback(frame, {1, 0x111}, {1, 1, 1}).value, "short feedback rejected");

    const auto read = DamiaoProtocol::encode_read_register(1, 0x15);
    check(read.value->id == 0x7FF && read.value->length == 4
        && read.value->data == std::array<std::uint8_t, 8>{1, 0, 0x33, 0x15, 0, 0, 0, 0}, "register read manual vector");
    const auto write = DamiaoProtocol::encode_write_register(1, 0x15, 12.5F);
    check(write.value && write.value->data == std::array<std::uint8_t, 8>{1, 0, 0x55, 0x15, 0, 0, 0x48, 0x41}, "register float write vector");
    const auto integer = DamiaoProtocol::encode_write_register(1, 0x09, std::uint32_t{0x12345678});
    check(integer.value->data == std::array<std::uint8_t, 8>{1, 0, 0x55, 9, 0x78, 0x56, 0x34, 0x12}, "register uint LE vector");
    check(!DamiaoProtocol::encode_write_register(1, 0x09, 1.0F).value, "register type mismatch");
    check(!DamiaoProtocol::encode_write_register(1, 0x0E, std::uint32_t{1}).value, "RO register rejected");
    check(!DamiaoProtocol::encode_write_register(1, 0x15, 0.0F).value, "mapping write zero rejected");
    check(!DamiaoProtocol::encode_read_register(1, 0x32).value, "old-only register not guessed");
    check(register_info(0x37)->type == RegisterType::Float && register_info(0x38)->type == RegisterType::Float
        && register_info(0x25)->type == RegisterType::UInt32, "new manual types");
    check(!DamiaoProtocol::encode_write_register(1, 0x0A, std::uint32_t{0}).value, "mode zero rejected");
    check(DamiaoProtocol::encode_write_register(1, 0x1D, 25.0F).status.code == ErrorCode::Unsupported, "TBD range refused");
    check(!DamiaoProtocol::encode_write_register(1, 0x23, std::uint32_t{5}).value, "CAN FD rate refused");
    check(!DamiaoProtocol::encode_write_register(1, 0x08, std::uint32_t{0x10001}).value, "ESC validation before narrowing");
    check(!DamiaoProtocol::encode_write_register(1, 0x02, 200.0F).value, "exclusive register upper bound");
    check(!DamiaoProtocol::encode_write_register(1, 0x03, 1.0F).value, "exclusive current bound");
    check(DamiaoProtocol::encode_write_register(1, 0x05, -1.0F).value.has_value(), "negative DEC valid");

    CanFrame reply{0x11, 8, {1, 0, 0x33, 0x15, 0, 0, 0x48, 0x41}, SteadyClock::now()};
    const RegisterReplyExpectation expected{{1, 0x11}, RegisterOperation::Read, 0x15};
    check(std::get<float>(*DamiaoProtocol::decode_register_reply(reply, expected).value) == 12.5F, "read reply vector");
    for (std::size_t field : {0, 1, 2, 3})
    {
        auto incorrect = reply;
        incorrect.data[field] ^= 1;
        check(!DamiaoProtocol::decode_register_reply(incorrect, expected).value, "all reply fields matched");
    }
    reply.data = {1, 0, 0x33, 0x15, 0, 0, 0x80, 0x7F};
    check(!DamiaoProtocol::decode_register_reply(reply, expected).value, "Inf reply rejected");
    const auto query = DamiaoProtocol::encode_state_query(1);
    check(query.value->length == 4 && query.value->data[2] == 0xCC && query.value->data[3] == 0, "independent status query");
    for (std::uint32_t mode = 1; mode <= 4; ++mode)
    {
        const auto command = DamiaoProtocol::encode_management_command(1, static_cast<ControlMode>(mode), ManagementCommand::Enable);
        check(command.value->id == 1 + (mode - 1) * 0x100
            && command.value->data == std::array<std::uint8_t, 8>{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}, "management mode offset");
    }
    check(!DamiaoProtocol::encode_management_command(1, static_cast<ControlMode>(0), ManagementCommand::Enable).value, "invalid mode rejected");
    const auto save = DamiaoProtocol::encode_save_parameters(1);
    check(save.value->length == 4 && save.value->data[2] == 0xAA && save.value->data[3] == 1, "flash request vector");
    reply = *save.value;
    reply.id = 0x11;
    check(DamiaoProtocol::decode_save_reply(reply, {1, 0x11}).code == ErrorCode::Ok, "four byte save ack");
    reply.length = 8;
    check(DamiaoProtocol::decode_save_reply(reply, {1, 0x11}).code == ErrorCode::Ok, "padded save ack");
    reply.length = 3;
    check(DamiaoProtocol::decode_save_reply(reply, {1, 0x11}).code == ErrorCode::InvalidFrame, "short save ack");
}

void test_native_frames()
{
    can_frame native{};
    native.can_id = 0x11;
    native.len = 8;
    const auto now = SteadyClock::now();
    check(detail::decode_native_frame(native, CAN_MTU, now).value->received_at == now, "native timestamp");
    for (std::size_t size : {std::size_t{0}, std::size_t{CAN_MTU - 1}, std::size_t{CANFD_MTU}})
    {
        check(!detail::decode_native_frame(native, size, now).value, "MTU exactness");
    }
    native.can_id |= CAN_EFF_FLAG;
    check(!detail::decode_native_frame(native, CAN_MTU, now).value, "EFF rejected");
    native.can_id = CAN_RTR_FLAG | 0x11;
    check(!detail::decode_native_frame(native, CAN_MTU, now).value, "RTR rejected");
    native.can_id = CAN_ERR_FLAG | 0x40;
    check(detail::decode_native_frame(native, CAN_MTU, now).status.code == ErrorCode::BusError, "bus-off error not decoded");
    native.can_id = 0x11;
    native.len = 9;
    check(!detail::decode_native_frame(native, CAN_MTU, now).value, "oversized native payload");
    CanFrame frame;
    frame.id = 0x800;
    check(!detail::encode_native_frame(frame).value, "native send ID check");
    frame.id = 1;
    frame.length = 9;
    check(!detail::encode_native_frame(frame).value, "native send length check");
    SocketCanTransport socket;
    check(!socket.is_open(), "constructor does not connect");
    check(socket.send(frame).code == ErrorCode::Disconnected, "closed send error");
    check(socket.receive(SteadyClock::now()).status.code == ErrorCode::Disconnected, "closed receive error");
    check(socket.open(TransportConfig{""}).code == ErrorCode::InvalidConfiguration, "invalid interface before syscall");
    check(socket.open(TransportConfig{std::string(16, 'x')}).code == ErrorCode::InvalidConfiguration, "long interface rejected");
    check(socket.close().code == ErrorCode::Ok && socket.close().code == ErrorCode::Ok, "close idempotent");
}

void test_ownership()
{
    char directory[] = "/tmp/damiao-core-test-XXXXXX";
    check(::mkdtemp(directory) != nullptr, "temporary lock directory");
    const std::string path = std::string(directory) + "/bus.lock";
    int first = -1;
    int second = -1;
    check(detail::acquire_lock(path, first).code == ErrorCode::Ok, "first owner acquired");
    check(detail::acquire_lock(path, second).code == ErrorCode::OwnershipConflict, "same process instance conflict");
    const pid_t child = ::fork();
    check(child >= 0, "fork owner test");
    if (child == 0)
    {
        ::close(first);
        int attempt = -1;
        const auto status = detail::acquire_lock(path, attempt);
        ::_exit(status.code == ErrorCode::OwnershipConflict ? 0 : 1);
    }
    int result = 0;
    check(::waitpid(child, &result, 0) == child && WIFEXITED(result) && WEXITSTATUS(result) == 0, "cross-process owner conflict");
    detail::release_lock(first);
    check(detail::acquire_lock(path, second).code == ErrorCode::Ok, "released owner reacquired");
    detail::release_lock(second);
    detail::release_lock(second);
    check(::unlink(path.c_str()) == 0 && ::rmdir(directory) == 0, "temporary ownership cleanup");
}

// 模拟后端仅存内存中的帧，不创建 CAN Socket；支持丢帧、错误与读回不一致注入。
class FakeTransport final : public ICanTransport
{
public:
    struct Motor
    {
        std::uint16_t esc;
        std::uint16_t mst;
        std::uint8_t status = 0;
        std::array<RegisterValue, 256> registers{};
    };

    FakeTransport()
    {
        for (std::uint16_t index = 1; index <= 2; ++index)
        {
            Motor motor{index, static_cast<std::uint16_t>(0x10 + index), 0, {}};
            motor.registers[0x08] = std::uint32_t{index};
            motor.registers[0x07] = std::uint32_t{motor.mst};
            motor.registers[0x0A] = std::uint32_t{2};
            motor.registers[0x0E] = std::uint32_t{72};
            motor.registers[0x15] = index == 1 ? 12.5F : 6.0F;
            motor.registers[0x16] = index == 1 ? 30.0F : 10.0F;
            motor.registers[0x17] = index == 1 ? 10.0F : 28.0F;
            motor.registers[0x09] = std::uint32_t{2000};
            motors_.push_back(motor);
        }
    }

    Status open(const TransportConfig& config) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        open_ = true;
        passive_ = config.passive;
        queue_.clear();
        return {};
    }

    Status close() override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        open_ = false;
        queue_.clear();
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
        if (passive_)
        {
            return {ErrorCode::OwnershipConflict, ""};
        }
        sent_.push_back(request);
        if (fail_send_ != 0 && sent_.size() == fail_send_)
        {
            return {ErrorCode::WouldBlock, ""};
        }
        if (drop_)
        {
            return {};
        }
        for (auto& motor : motors_)
        {
            if (request.id == 0x7FF && request.data[0] == motor.esc)
            {
                const auto operation = request.data[2];
                const auto rid = request.data[3];
                if (operation == 0xCC)
                {
                    auto response = feedback(motor.esc, motor.mst, motor.status);
                    if (ambiguous_state_)
                    {
                        response.data = {static_cast<std::uint8_t>(motor.esc), 0, 0x33, 0x15, 0, 0, 0x48, 0x41};
                    }
                    queue_.push_back(response);
                }
                else
                {
                    CanFrame response = request;
                    response.id = motor.mst;
                    response.length = operation == 0xAA ? 4 : 8;
                    if (operation == 0x55)
                    {
                        std::uint32_t bits = std::uint32_t(request.data[4]) | (std::uint32_t(request.data[5]) << 8)
                            | (std::uint32_t(request.data[6]) << 16) | (std::uint32_t(request.data[7]) << 24);
                        if (register_info(rid)->type == RegisterType::Float)
                        {
                            float number;
                            std::memcpy(&number, &bits, sizeof(number));
                            motor.registers[rid] = number;
                        }
                        else
                        {
                            motor.registers[rid] = bits;
                        }
                        if (corrupt_readback_ && rid == 0x09)
                        {
                            motor.registers[rid] = std::uint32_t{999};
                        }
                    }
                    else if (operation == 0x33)
                    {
                        const auto& value = motor.registers[rid];
                        std::uint32_t bits;
                        if (const auto* number = std::get_if<float>(&value))
                        {
                            std::memcpy(&bits, number, sizeof(bits));
                        }
                        else
                        {
                            bits = std::get<std::uint32_t>(value);
                        }
                        for (unsigned int byte = 0; byte < 4; ++byte)
                        {
                            response.data[4 + byte] = static_cast<std::uint8_t>(bits >> (8 * byte));
                        }
                    }
                    response.received_at = SteadyClock::now();
                    if (!(drop_write_ack_ && operation == 0x55))
                    {
                        queue_.push_back(response);
                    }
                }
            }
            else if ((request.id & 0xFF) == motor.esc)
            {
                if (request.data[0] == 0xFF && request.data[7] >= 0xFB)
                {
                    motor.status = request.data[7] == 0xFC ? 1 : 0;
                }
                else
                {
                    queue_.push_back(feedback(motor.esc, motor.mst, motor.status));
                }
            }
        }
        ready_.notify_all();
        return {};
    }

    Result<CanFrame> receive(Deadline until) override
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!ready_.wait_until(lock, until, [this]
        {
            return !queue_.empty() || !open_ || error_ != ErrorCode::Ok;
        }))
        {
            return {{ErrorCode::Timeout, ""}, std::nullopt};
        }
        if (error_ != ErrorCode::Ok)
        {
            return {{error_, ""}, std::nullopt};
        }
        if (!open_)
        {
            return {{ErrorCode::Disconnected, ""}, std::nullopt};
        }
        auto response = queue_.front();
        queue_.pop_front();
        return {{}, response};
    }

    void inject(CanFrame frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push_back(frame);
        ready_.notify_all();
    }

    std::size_t sent_count() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return sent_.size();
    }

    void set_drop(bool drop)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        drop_ = drop;
    }

    void fail_send(std::size_t number)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        fail_send_ = number;
    }

    void set_error(ErrorCode error)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = error;
        ready_.notify_all();
    }

    void corrupt_readback()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        corrupt_readback_ = true;
    }

    void drop_write_ack()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        drop_write_ack_ = true;
    }

    RegisterValue register_value(std::size_t index, std::uint8_t rid)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return motors_[index].registers[rid];
    }

    void ambiguous_state()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ambiguous_state_ = true;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    bool open_ = false;
    bool passive_ = false;
    bool drop_ = false;
    bool corrupt_readback_ = false;
    bool ambiguous_state_ = false;
    bool drop_write_ack_ = false;
    ErrorCode error_ = ErrorCode::Ok;
    std::size_t fail_send_ = 0;
    std::vector<Motor> motors_;
    std::deque<CanFrame> queue_;
    std::vector<CanFrame> sent_;
};

MotorConfig motor_config(std::uint16_t esc)
{
    MotorConfig config;
    config.name = "motor_" + std::to_string(esc);
    config.model = "J4310P-2EC";
    config.address = {esc, static_cast<std::uint16_t>(0x10 + esc)};
    config.mapping = {12.5, 30, 10};
    config.mapping_confirmed = true;
    config.min_output_position_rad = -2.0;
    config.max_output_position_rad = 2.0;
    config.max_output_speed_rad_s = 1.0;
    return config;
}

BusConfig bus_config(bool passive = false)
{
    BusConfig config;
    config.transport.passive = passive;
    // 仅软件模拟的测试期限，不作为实体机械臂保护参数。
    config.feedback_timeout = 500ms;
    config.management_quiet_period = 1ms;
    return config;
}

template<typename Predicate>
void eventually(Predicate predicate, const char* message)
{
    const auto until = deadline();
    while (SteadyClock::now() < until)
    {
        if (predicate())
        {
            return;
        }
        std::this_thread::sleep_for(1ms);
    }
    throw std::runtime_error(message);
}

void test_registration()
{
    DamiaoBus bus(std::make_unique<FakeTransport>());
    check(bus.register_motor(motor_config(1)).value == 0, "stable motor index");
    check(!bus.register_motor(motor_config(1)).value, "duplicate ESC rejected");
    auto config = motor_config(2);
    config.address.mst_id = 0x11;
    check(!bus.register_motor(config).value, "duplicate MST rejected");
    config.address.mst_id = 0x101;
    check(!bus.register_motor(config).value, "cross motor low byte conflict");
    config.address.mst_id = 0x800;
    check(!bus.register_motor(config).value, "out of range MST rejected");
    config = motor_config(2);
    config.name = "motor_1";
    check(!bus.register_motor(config).value, "duplicate name rejected");
    for (std::uint16_t id = 2; id <= 6; ++id)
    {
        check(bus.register_motor(motor_config(id)).value.has_value(), "six motors supported");
    }
    check(!bus.register_motor(motor_config(7)).value, "seventh motor rejected");
    check(bus.open(BusConfig{}).code == ErrorCode::InvalidConfiguration, "timeouts not invented");
    check(bus.open(bus_config()).code == ErrorCode::Ok, "bus open");
    check(!bus.register_motor(motor_config(7)).value, "no registration during receive");
    check(bus.snapshot(0).status.code == ErrorCode::StaleFeedback, "no feedback not zero success");
    check(!bus.snapshot(6).value, "snapshot index check");
    check(bus.close().code == ErrorCode::Ok && bus.close().code == ErrorCode::Ok, "bus close idempotent");
}

void test_maintenance_and_control()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    DamiaoBus bus(std::move(transport));
    bus.register_motor(motor_config(1));
    bus.register_motor(motor_config(2));
    check(bus.open(bus_config()).code == ErrorCode::Ok && fake->sent_count() == 0, "open has no implicit sends");
    check(bus.begin_control().code != ErrorCode::Ok, "unsynchronized control rejected");
    check(bus.synchronize_motor(0, deadline()).code == ErrorCode::Ok, "motor 1 synchronized");
    check(bus.synchronize_motor(1, deadline()).code == ErrorCode::Ok, "motor 2 synchronized");
    check(near(bus.motor_config(0).value->mapping.position_rad, 12.5)
        && near(bus.motor_config(1).value->mapping.position_rad, 6), "same model independent mappings");
    check(bus.query_state(0, deadline()).status.code == ErrorCode::Ok, "motor 1 queried");
    check(bus.query_state(1, deadline()).status.code == ErrorCode::Ok, "motor 2 queried");
    const auto before_write = fake->sent_count();
    const auto report = bus.write_parameter_verified(0, 0x09, std::uint32_t{2500}, deadline());
    check(report.status.code == ErrorCode::Ok && report.verified && report.write_sent
        && report.previous == std::optional<RegisterValue>{std::uint32_t{2000}}
        && report.readback == std::optional<RegisterValue>{std::uint32_t{2500}}, "verified write evidence");
    check(fake->sent_count() == before_write + 3, "read write read sequence, no flash");
    check(bus.snapshot(0).status.code == ErrorCode::StaleFeedback, "configuration write invalidates cache");
    bus.query_state(0, deadline());
    const auto rejected_count = fake->sent_count();
    check(bus.write_parameter_verified(0, 0x08, std::uint32_t{3}, deadline()).status.code == ErrorCode::Unsupported, "ID migration not blindly executed");
    check(fake->sent_count() == rejected_count, "unsupported migration no send");
    check(bus.save_parameters(0, SteadyClock::now() + 10ms).code == ErrorCode::InvalidCommand, "flash deadline budget");
    check(bus.save_parameters(0, deadline()).code == ErrorCode::Ok, "flash ack matched");
    const auto command_count = fake->sent_count();
    check(bus.enable(0, SteadyClock::now()).code == ErrorCode::Timeout && fake->sent_count() == command_count,
        "expired management command no send");
    check(bus.switch_mode(0, ControlMode::PositionVelocity, deadline()).code == ErrorCode::Ok, "mode verified write");
    check(bus.snapshot(0).status.code == ErrorCode::StaleFeedback, "mode invalidates state");
    bus.query_state(0, deadline());
    const auto revision = bus.snapshot(0).value->mapping_revision;
    check(bus.save_zero(0, deadline()).code == ErrorCode::Ok, "explicit zero command");
    bus.query_state(0, deadline());
    check(bus.snapshot(0).value->mapping_revision > revision, "zero invalidates coordinate revision");
    const auto queued_old_mapping = feedback(1, 0x11, 0);
    const auto mapped = bus.write_parameter_verified(0, 0x15, 8.0F, deadline());
    check(mapped.verified && near(bus.motor_config(0).value->mapping.position_rad, 8.0)
        && near(bus.motor_config(1).value->mapping.position_rad, 6.0), "mapping write updates only target");
    const auto rejected_before = bus.diagnostics().rejected_frames;
    fake->inject(queued_old_mapping);
    eventually([&]
    {
        return bus.diagnostics().rejected_frames > rejected_before;
    }, "old mapping queue rejected after configuration commit");
    check(bus.snapshot(0).status.code == ErrorCode::StaleFeedback, "old queue cannot revive new mapping cache");
    bus.query_state(0, deadline());
    check(bus.enable(0, deadline()).code == ErrorCode::Ok && bus.enable(1, deadline()).code == ErrorCode::Ok, "explicit enables confirmed");
    const auto enabled_count = fake->sent_count();
    check(bus.read_parameter(0, 0x0A, deadline()).status.code == ErrorCode::InvalidCommand, "no parameter query among enabled motors");
    check(fake->sent_count() == enabled_count, "maintenance gate before sending");
    check(bus.begin_control().code == ErrorCode::Ok, "explicit control permit");
    check(bus.query_state(0, deadline()).status.code == ErrorCode::InvalidCommand, "no query interleaving in control");
    check(bus.close().code == ErrorCode::InvalidCommand, "active close rejected");
    std::array<PositionVelocityCommand, 2> commands{{{0.0, 0.5}, {0.0, 0.5}}};
    std::array<ErrorCode, 2> results;
    commands[1].output_position_rad = 3;
    const auto before_invalid = fake->sent_count();
    check(bus.send_position_velocity_batch(commands.data(), 2, results.data()) == ErrorCode::InvalidCommand, "batch limit rejection");
    check(fake->sent_count() == before_invalid && results[0] == ErrorCode::NotExecuted, "whole batch checked first");
    commands[1].output_position_rad = 0;
    ErrorCode sent = ErrorCode::WouldBlock;
    eventually([&]
    {
        sent = bus.send_position_velocity_batch(commands.data(), 2, results.data());
        return sent != ErrorCode::WouldBlock;
    }, "batch send eventually available");
    check(sent == ErrorCode::Ok && results[0] == ErrorCode::Ok && results[1] == ErrorCode::Ok, "batch results");
    fake->fail_send(fake->sent_count() + 2);
    eventually([&]
    {
        sent = bus.send_position_velocity_batch(commands.data(), 2, results.data());
        return sent != ErrorCode::WouldBlock;
    }, "partial failure exercised");
    check(sent == ErrorCode::PartialFailure && results[0] == ErrorCode::Ok
        && results[1] == ErrorCode::WouldBlock && bus.state() == BusState::Fault, "partial batch latches fault");
    const auto fault_count = fake->sent_count();
    check(bus.send_position_velocity_batch(commands.data(), 2, results.data()) != ErrorCode::Ok
        && fake->sent_count() == fault_count, "no automatic retry after partial send");
    check(bus.diagnostics().send_failures == 1, "send failure diagnostics");
    check(bus.disable(0, deadline()).code == ErrorCode::Ok && bus.disable(1, deadline()).code == ErrorCode::Ok,
        "explicit best-effort disable after partial failure");
    check(bus.state() == BusState::Fault, "disable does not auto-clear fault");
    check(bus.close().code == ErrorCode::Ok, "fault resource cleanup");
}

void test_passive_routing()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    DamiaoBus bus(std::move(transport));
    bus.register_motor(motor_config(1));
    bus.register_motor(motor_config(2));
    auto config = bus_config(true);
    config.feedback_timeout = 50ms;
    check(bus.open(config).code == ErrorCode::Ok, "passive open");
    fake->inject(feedback(1, 0x11, 0));
    fake->inject(feedback(2, 0x12, 1));
    eventually([&]
    {
        return bus.snapshot(0).status.code == ErrorCode::Ok && bus.snapshot(1).status.code == ErrorCode::Ok;
    }, "two independent routes");
    check(bus.read_parameter(0, 0x15, deadline()).status.code != ErrorCode::Ok
        && bus.query_state(0, deadline()).status.code != ErrorCode::Ok && fake->sent_count() == 0, "passive no active query");
    auto ordinary = feedback(1, 0x11, 0);
    ordinary.data[2] = 0x33;
    ordinary.data[3] = 0x15;
    fake->inject(ordinary);
    eventually([&]
    {
        return bus.snapshot(0).value->received_at == ordinary.received_at;
    }, "ordinary data containing opcode stays feedback");
    const auto sequence = bus.snapshot(0).value->sequence;
    auto old = ordinary;
    old.received_at -= 5ms;
    old.data[1] = 0xFF;
    fake->inject(old);
    fake->inject(ordinary);
    auto wrong_id = feedback(2, 0x11, 1);
    fake->inject(wrong_id);
    std::this_thread::sleep_for(5ms);
    check(bus.snapshot(0).value->sequence == sequence, "duplicate old and wrong payload do not refresh");
    fake->inject(feedback(2, 0x12, 1));
    std::this_thread::sleep_for(55ms);
    fake->inject(feedback(2, 0x12, 1));
    eventually([&]
    {
        return bus.snapshot(1).status.code == ErrorCode::Ok;
    }, "motor 2 refreshed");
    check(bus.snapshot(0).status.code == ErrorCode::StaleFeedback, "other axis cannot refresh stale motor");
    std::array<MotorState, 2> states;
    ErrorCode code = ErrorCode::WouldBlock;
    eventually([&]
    {
        code = bus.snapshot_into(states.data(), states.size());
        return code != ErrorCode::WouldBlock;
    }, "preallocated snapshot available");
    check(code == ErrorCode::StaleFeedback && !states[0].valid && states[1].valid, "snapshot_into freshness");
    const auto before = SteadyClock::now();
    bus.close();
    check(SteadyClock::now() - before < 100ms, "bounded receiver exit");
}

void test_transaction_faults()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    DamiaoBus bus(std::move(transport));
    bus.register_motor(motor_config(1));
    bus.open(bus_config());
    fake->set_drop(true);
    const auto expired_count = fake->sent_count();
    check(bus.read_parameter(0, 0x15, SteadyClock::now()).status.code == ErrorCode::Timeout
        && fake->sent_count() == expired_count, "expired deadline no send");
    const auto timeout = bus.read_parameter(0, 0x15, SteadyClock::now() + 20ms);
    check(timeout.status.code == ErrorCode::Timeout && !timeout.value && bus.state() == BusState::Fault, "dropped reply timeout fault");
    check(bus.diagnostics().transaction_timeouts == 1, "timeout diagnostics");
    const auto count = fake->sent_count();
    CanFrame late{0x11, 8, {1, 0, 0x33, 0x15, 0, 0, 0x48, 0x41}, SteadyClock::now()};
    fake->inject(late);
    check(bus.read_parameter(0, 0x15, deadline()).status.code != ErrorCode::Ok
        && fake->sent_count() == count, "late reply not used for retry in same session");
    bus.close();
    fake->set_drop(false);
    bus.open(bus_config());
    check(bus.synchronize_motor(0, deadline()).code == ErrorCode::Ok, "explicit reopen resynchronizes");
    check(bus.query_state(0, deadline()).status.code == ErrorCode::Ok, "state after reopen");
    fake->corrupt_readback();
    const auto report = bus.write_parameter_verified(0, 0x09, std::uint32_t{2500}, deadline());
    check(report.write_sent && !report.verified && report.readback == std::optional<RegisterValue>{std::uint32_t{999}}
        && report.status.code == ErrorCode::AmbiguousReply && bus.state() == BusState::Fault, "write readback mismatch not success");
    bus.close();
}

void test_concurrent_management()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    DamiaoBus bus(std::move(transport));
    bus.register_motor(motor_config(1));
    bus.open(bus_config());
    fake->set_drop(true);
    Result<RegisterValue> first;
    std::thread manager([&]
    {
        first = bus.read_parameter(0, 0x15, SteadyClock::now() + 100ms);
    });
    // 等待首个事务实际发出，第二个请求应立即返回忙，不越过自身截止时间。
    const auto until = deadline();
    while (fake->sent_count() == 0 && SteadyClock::now() < until)
    {
        std::this_thread::sleep_for(1ms);
    }
    const auto before = SteadyClock::now();
    const auto second = bus.read_parameter(0, 0x16, before + 5ms);
    const auto elapsed = SteadyClock::now() - before;
    const auto close_busy = bus.close();
    CanFrame stale{0x11, 8, {1, 0, 0x33, 0x15, 0, 0, 0x48, 0x41}, before - 1s};
    fake->inject(stale);
    manager.join();
    check(second.status.code == ErrorCode::WouldBlock && elapsed < 20ms, "concurrent management bounded busy result");
    check(close_busy.code == ErrorCode::WouldBlock, "close does not block behind management deadline");
    check(first.status.code == ErrorCode::Timeout && !first.value && fake->sent_count() == 1,
        "stale queued reply not associated with new transaction");
    bus.close();
}

void test_actual_position_violation()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    DamiaoBus bus(std::move(transport));
    bus.register_motor(motor_config(1));
    bus.open(bus_config());
    bus.synchronize_motor(0, deadline());
    bus.query_state(0, deadline());
    bus.enable(0, deadline());
    check(bus.begin_control().code == ErrorCode::Ok, "position violation test enters control");
    auto out_of_range = feedback(1, 0x11, 1);
    out_of_range.data[1] = 0xFF;
    out_of_range.data[2] = 0xFF;
    fake->inject(out_of_range);
    eventually([&]
    {
        return bus.state() == BusState::Fault;
    }, "actual position limit faults even with raw enabled status");
    check(bus.diagnostics().last_error == ErrorCode::InvalidCommand, "actual limit diagnostic");
    const auto count = fake->sent_count();
    PositionVelocityCommand command{0.0, 0.5};
    ErrorCode result;
    check(bus.send_position_velocity_batch(&command, 1, &result) != ErrorCode::Ok
        && fake->sent_count() == count, "no commands after actual position violation");
    bus.close();
}

void test_lost_write_ack_and_explicit_clear()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    DamiaoBus bus(std::move(transport));
    bus.register_motor(motor_config(1));
    bus.open(bus_config());
    bus.synchronize_motor(0, deadline());
    bus.query_state(0, deadline());
    const auto sequence = bus.snapshot(0).value->sequence;
    fake->inject(feedback(1, 0x11, 3));
    eventually([&]
    {
        return bus.snapshot(0).value->sequence > sequence;
    }, "calibration fault received");
    check(bus.snapshot(0).status.code == ErrorCode::MotorFault, "fault feedback not normal");
    const auto before_enable = fake->sent_count();
    check(bus.enable(0, deadline()).code == ErrorCode::InvalidCommand && fake->sent_count() == before_enable,
        "fault does not automatically clear or enable");
    check(bus.clear_error(0, deadline()).code == ErrorCode::Ok && bus.snapshot(0).value->raw_status == 0,
        "explicit clear separated from enable");
    fake->drop_write_ack();
    const auto before_write = fake->sent_count();
    const auto report = bus.write_parameter_verified(0, 0x09, std::uint32_t{3000}, SteadyClock::now() + 30ms);
    check(report.write_sent && !report.verified && !report.readback && report.status.code == ErrorCode::Timeout,
        "lost write acknowledgement remains unknown");
    check(fake->register_value(0, 0x09) == RegisterValue{std::uint32_t{3000}}
        && fake->sent_count() == before_write + 2, "timeout may mean device already executed, no blind retry");
    bus.close();
}

void test_ambiguous_query_and_disconnect()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    DamiaoBus bus(std::move(transport));
    bus.register_motor(motor_config(1));
    bus.open(bus_config());
    bus.synchronize_motor(0, deadline());
    fake->ambiguous_state();
    check(bus.query_state(0, deadline()).status.code == ErrorCode::AmbiguousReply, "fully overlapping state not trusted");
    check(bus.state() == BusState::Fault, "ambiguity fault latched");
    bus.close();
    bus.open(bus_config());
    fake->set_error(ErrorCode::Disconnected);
    eventually([&]
    {
        return bus.state() == BusState::Fault;
    }, "USB disconnected fault");
    bus.close();
}

}  // namespace

int main()
{
    const std::pair<const char*, std::function<void()>> tests[] =
    {
        {"protocol vectors and validation", test_protocol},
        {"native frame validation", test_native_frames},
        {"process ownership", test_ownership},
        {"motor registration", test_registration},
        {"maintenance and batch control", test_maintenance_and_control},
        {"passive routing and stale feedback", test_passive_routing},
        {"transaction timeout and readback mismatch", test_transaction_faults},
        {"concurrent management deadlines", test_concurrent_management},
        {"actual position limit fault", test_actual_position_violation},
        {"lost write ack and explicit clear", test_lost_write_ack_and_explicit_clear},
        {"ambiguous query and disconnect", test_ambiguous_query_and_disconnect}
    };
    for (const auto& test : tests)
    {
        try
        {
            test.second();
            std::cout << "PASS: " << test.first << '\n';
        }
        catch (const std::exception& error)
        {
            std::cerr << "FAIL: " << test.first << ": " << error.what() << '\n';
            return 1;
        }
    }
    return 0;
}
