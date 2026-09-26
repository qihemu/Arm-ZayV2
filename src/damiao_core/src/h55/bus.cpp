#include <damiao_core/h55/bus.hpp>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <thread>
namespace damiao::h55
{
H55Bus::H55Bus(BusConfig config, std::unique_ptr<ICanTransport> transport)
    : config_(config), transport_(std::move(transport))
{
    if (!transport_ || config_.reply_timeout.count() <= 0 || config_.feedback_timeout.count() <= 0 ||
        !std::isfinite(config_.readback_tolerance) || config_.readback_tolerance < 0 ||
        !std::isfinite(config_.min_bus_voltage) || !std::isfinite(config_.max_bus_voltage) ||
        config_.min_bus_voltage <= 0 || config_.max_bus_voltage <= config_.min_bus_voltage ||
        !std::isfinite(config_.max_start_temperature) || config_.max_start_temperature <= 0)
    {
        throw std::invalid_argument("Invalid H55 transport/timeout");
    }
    for (const auto &m : config_.motors)
    {
        if (DamiaoProtocol::validate_address(m.address).code != ErrorCode::Ok ||
            DamiaoProtocol::validate_mapping_limits(m.mapping).code != ErrorCode::Ok ||
            !std::isfinite(m.maximum_speed) || m.maximum_speed <= 0 || m.timeout_raw == 0)
        {
            throw std::invalid_argument("Invalid H55 configuration");
        }
    }
    if (config_.motors[0].address.esc_id == config_.motors[1].address.esc_id ||
        config_.motors[0].address.mst_id == config_.motors[1].address.mst_id)
    {
        throw std::invalid_argument("Duplicate H55 address");
    }
}
Status H55Bus::open(const TransportConfig &config)
{
    return transport_->open(config);
}
void H55Bus::close()
{
    transport_->close();
}
bool H55Bus::identities_verified() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return verified_;
}
Status H55Bus::receive_once(Deadline deadline)
{
    const auto result = transport_->receive(deadline);
    if (result.status.code == ErrorCode::Ok && result.value)
    {
        route(*result.value);
    }
    return result.status;
}
void H55Bus::route(const CanFrame &frame)
{
    std::lock_guard<std::mutex> lock(mutex_);
    // 管理回应先分流，不能误解码成普通反馈。
    if (is_register_frame(frame))
    {
        if (pending_ && frame.received_at >= pending_since_ &&
            frame.received_at <= pending_since_ + config_.reply_timeout &&
            decode_register(frame, config_.motors[pending_motor_].address, pending_rid_).value)
        {
            reply_ = frame;
            pending_ = false;
            changed_.notify_all();
        }
        return;
    }
    for (std::size_t i = 0; i < 2; ++i)
    {
        if (frame.id != config_.motors[i].address.mst_id)
        {
            continue;
        }
        auto decoded =
            DamiaoProtocol::decode_feedback(frame, config_.motors[i].address, config_.motors[i].mapping);
        if (decoded.value && frame.received_at > state_[i].received_at)
        {
            decoded.value->sequence = state_[i].sequence + 1;
            state_[i] = *decoded.value;
            changed_.notify_all();
        }
    }
}
std::array<MotorState, 2> H55Bus::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}
Status H55Bus::transmit(const Result<CanFrame> &frame)
{
    if (!frame.value)
    {
        return frame.status;
    }
    return transport_->send(*frame.value);
}
Result<RegisterValue> H55Bus::read_register(std::size_t index, std::uint8_t rid)
{
    if (index >= 2)
    {
        return {{ErrorCode::InvalidCommand, "Invalid wheel index"}, std::nullopt};
    }
    const auto request = encode_read(config_.motors[index].address.esc_id, rid);
    if (!request.value)
    {
        return {request.status, std::nullopt};
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_ = true;
        pending_motor_ = index;
        pending_rid_ = rid;
        pending_since_ = SteadyClock::now();
    }
    const auto sent = transmit(request);
    std::unique_lock<std::mutex> lock(mutex_);
    if (sent.code != ErrorCode::Ok)
    {
        pending_ = false;
        return {sent, std::nullopt};
    }
    if (!changed_.wait_until(lock, pending_since_ + config_.reply_timeout, [this] { return !pending_; }))
    {
        pending_ = false;
        return {{ErrorCode::Timeout, "H55 register reply timeout"}, std::nullopt};
    }
    return decode_register(reply_, config_.motors[index].address, rid);
}
Status H55Bus::verify_configuration()
{
    // 只读核验；整个身份/模式通过前不能发送速度或管理特殊帧。
    {
        std::lock_guard<std::mutex> lock(mutex_);
        verified_ = false;
        protection_verified_ = false;
    }
    for (std::size_t i = 0; i < 2; ++i)
    {
        const auto &m = config_.motors[i];
        const std::pair<std::uint8_t, double> expected[] = {{8, double(m.address.esc_id)},
                                                            {7, double(m.address.mst_id)},
                                                            {10, 3},
                                                            {0x15, m.mapping.position_rad},
                                                            {0x16, m.mapping.velocity_rad_s},
                                                            {0x17, m.mapping.torque_nm},
                                                            {0x23, 4},
                                                            {0x0e, double(m.firmware)},
                                                            {0x24, double(m.sub_version)}};
        for (const auto &item : expected)
        {
            const auto result = read_register(i, item.first);
            if (!result.value)
            {
                return result.status;
            }
            const double value = std::visit([](auto v) { return double(v); }, *result.value);
            const double tolerance = std::holds_alternative<std::uint32_t>(*result.value)
                                         ? 0.0
                                         : config_.readback_tolerance * std::max(1.0, std::abs(item.second));
            if (std::abs(value - item.second) > tolerance)
            {
                return {ErrorCode::InvalidConfiguration, "H55 readback mismatch wheel=" + std::to_string(i) +
                                                             " rid=" + std::to_string(item.first)};
            }
        }
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        verified_ = true; // 身份可信后，保护不符也允许停止，但不允许使能。
    }
    const auto stopped = disable_pair();
    if (stopped.code != ErrorCode::Ok)
    {
        return stopped;
    }
    for (std::size_t i = 0; i < 2; ++i)
    {
        for (auto rid : {0x3c, 0x3d, 0x3e})
        {
            const auto r = read_register(i, rid);
            if (!r.value)
            {
                return r.status;
            }
            const double v = std::visit([](auto x) { return double(x); }, *r.value);
            if ((rid == 0x3c && (v < config_.min_bus_voltage || v > config_.max_bus_voltage)) ||
                (rid != 0x3c && (v < 0 || v > config_.max_start_temperature)))
            {
                return {ErrorCode::InvalidConfiguration, "H55 startup voltage/temperature guard"};
            }
        }
    }
    for (std::size_t i = 0; i < 2; ++i)
    {
        for (const auto rid : {0x09, 0x06})
        {
            const auto result = read_register(i, rid);
            if (!result.value)
            {
                return result.status;
            }
            const double before = std::visit([](auto v) { return double(v); }, *result.value);
            const double expected =
                rid == 9 ? config_.motors[i].timeout_raw : config_.motors[i].maximum_speed;
            double value = before;
            if (config_.write_protection_on_startup &&
                std::abs(before - expected) > config_.readback_tolerance * std::max(1.0, expected))
            {
                // Refresh both disabled acknowledgements before each individual write.
                const auto disabled = disable_pair();
                if (disabled.code != ErrorCode::Ok)
                {
                    return disabled;
                }
                std::cerr << "H55 protection write wheel=" << i << " rid=" << int(rid)
                          << " before=" << before << " requested=" << expected << std::endl;
                const auto written = transmit(encode_protection_write(config_.motors[i].address.esc_id, rid, expected));
                if (written.code != ErrorCode::Ok)
                {
                    return written;
                }
                const auto readback = read_register(i, rid);
                if (!readback.value)
                {
                    return {readback.status.code, "Protection readback unknown wheel=" + std::to_string(i) +
                            " rid=" + std::to_string(rid) + ": " + readback.status.message};
                }
                value = std::visit([](auto v) { return double(v); }, *readback.value);
            }
            std::cerr << "H55 protection verified-read wheel=" << i << " rid=" << int(rid)
                      << " actual=" << value << " requested=" << expected << std::endl;
            if (std::abs(value - expected) > config_.readback_tolerance * std::max(1.0, expected))
            {
                return {ErrorCode::InvalidConfiguration, "H55 protection mismatch; configure while disabled"};
            }
        }
    }
    // 读参结束重新取新鲜失能反馈，避免初始化读取时间被算作反馈失联。
    const auto final_disabled = disable_pair();
    if (final_disabled.code != ErrorCode::Ok)
    {
        return final_disabled;
    }
    protection_verified_ = true;
    return {};
}
Status H55Bus::send_zero_unchecked()
{
    Status failure;
    for (const auto &m : config_.motors)
    {
        const auto result = transmit(encode_velocity(m.address.esc_id, 0));
        if (result.code != ErrorCode::Ok)
        {
            failure = result;
        }
    }
    return failure;
}
Status H55Bus::special_pair(ManagementCommand command, std::uint8_t expected)
{
    if (!identities_verified())
    {
        return {ErrorCode::InvalidConfiguration, "H55 identity not verified"};
    }
    if (command == ManagementCommand::Enable)
    {
        const auto zero = send_zero_unchecked();
        if (zero.code != ErrorCode::Ok)
        {
            return zero;
        }
    }
    const auto start = SteadyClock::now();
    Status failure;
    // 先通知两轮再等待，两轮不是CAN原子同步，部分发送必须报告。
    for (const auto &m : config_.motors)
    {
        const auto result = transmit(
            DamiaoProtocol::encode_management_command(m.address.esc_id, ControlMode::Velocity, command));
        if (result.code != ErrorCode::Ok)
        {
            failure = result;
        }
    }
    if (failure.code != ErrorCode::Ok)
    {
        return failure;
    }
    auto refresh = start;
    while (SteadyClock::now() < start + config_.reply_timeout)
    {
        const auto states = snapshot();
        bool confirmed = true;
        for (const auto &s : states)
        {
            confirmed = confirmed && s.valid && s.received_at >= start && s.raw_status == expected;
        }
        if (confirmed)
        {
            return {};
        }
        // FD可幂等重发；只重试未确认的轮子，不延长整个管理事务的截止时间。
        if (command == ManagementCommand::Disable &&
            SteadyClock::now() - refresh >= std::chrono::milliseconds(20))
        {
            for (std::size_t i = 0; i < 2; ++i)
            {
                if (states[i].valid && states[i].received_at >= start && states[i].raw_status == expected)
                {
                    continue;
                }
                const auto retry = transmit(DamiaoProtocol::encode_management_command(
                    config_.motors[i].address.esc_id, ControlMode::Velocity, command));
                if (retry.code != ErrorCode::Ok)
                {
                    return retry;
                }
            }
            refresh = SteadyClock::now();
        }
        // 使能确认期间保持零速刷新，不能让先使能的轮子超时。
        if (command == ManagementCommand::Enable &&
            SteadyClock::now() - refresh >= std::chrono::milliseconds(10))
        {
            const auto zero = send_zero_unchecked();
            if (zero.code != ErrorCode::Ok)
            {
                return zero;
            }
            refresh = SteadyClock::now();
        }
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait_for(lock, std::chrono::milliseconds(1));
    }
    std::string detail = "H55 state confirmation timeout expected=" + std::to_string(expected);
    const auto final = snapshot();
    for (std::size_t i = 0; i < 2; ++i)
    {
        detail += " wheel=" + std::to_string(config_.motors[i].address.esc_id) +
                  " status=" + std::to_string(final[i].raw_status) +
                  " new_reply=" + std::to_string(final[i].valid && final[i].received_at >= start);
    }
    return {ErrorCode::Timeout, detail};
}
Status H55Bus::enable_pair()
{
    if (!protection_verified_)
    {
        return {ErrorCode::InvalidConfiguration, "H55 protection not verified"};
    }
    return special_pair(ManagementCommand::Enable, 1);
}
Status H55Bus::disable_pair()
{
    return special_pair(ManagementCommand::Disable, 0);
}
Status H55Bus::poll_disabled_pair()
{
    // 身份未经核验不得向猜测地址发指令；发送成功不等于已收到失能确认。
    if (!identities_verified())
    {
        return {ErrorCode::InvalidConfiguration, "H55 identity not verified"};
    }
    Status failure;
    for (const auto &m : config_.motors)
    {
        const auto sent = transmit(DamiaoProtocol::encode_management_command(
            m.address.esc_id, ControlMode::Velocity, ManagementCommand::Disable));
        if (sent.code != ErrorCode::Ok)
        {
            failure = sent;
        }
    }
    return failure;
}
Status H55Bus::clear_pair()
{
    if (!identities_verified())
    {
        return {ErrorCode::InvalidConfiguration, "Identity unknown"};
    }
    for (const auto &m : config_.motors)
    {
        const auto result = transmit(DamiaoProtocol::encode_management_command(
            m.address.esc_id, ControlMode::Velocity, ManagementCommand::ClearError));
        if (result.code != ErrorCode::Ok)
        {
            return result;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return disable_pair();
}
Status H55Bus::send_velocity_pair(const std::array<double, 2> &speed)
{
    if (!identities_verified())
    {
        return {ErrorCode::InvalidConfiguration, "Identity unknown"};
    }
    const auto states = snapshot();
    for (std::size_t i = 0; i < 2; ++i)
    {
        if (!std::isfinite(speed[i]) || std::abs(speed[i]) > config_.motors[i].maximum_speed)
        {
            return {ErrorCode::InvalidCommand, "H55 speed limit"};
        }
        if (!states[i].valid || states[i].raw_status != 1 ||
            SteadyClock::now() - states[i].received_at > config_.feedback_timeout)
        {
            return {ErrorCode::StaleFeedback, "H55 enabled feedback unavailable"};
        }
    }
    for (std::size_t i = 0; i < 2; ++i)
    {
        const auto result = transmit(encode_velocity(config_.motors[i].address.esc_id, speed[i]));
        if (result.code != ErrorCode::Ok)
        {
            stop_pair();
            return {ErrorCode::PartialFailure, "H55 pair transmission incomplete"};
        }
    }
    return {};
}
Status H55Bus::stop_pair()
{
    if (!identities_verified())
    {
        return {ErrorCode::InvalidConfiguration, "No trusted stop address"};
    }
    send_zero_unchecked(); // 即使第一帧发送失败仍尝试两轮FD。
    return disable_pair();
}
} // namespace damiao::h55
