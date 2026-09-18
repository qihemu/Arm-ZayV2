#include "motor_scanner.hpp"

#include <damiao_core/protocol.hpp>
#include <damiao_core/transport.hpp>

#include <array>
#include <chrono>
#include <utility>

namespace damiao_tools
{
namespace
{

using namespace std::chrono_literals;

constexpr auto management_quiet_period = 5ms;

damiao::Status invalid_scan(const char* message)
{
    return {damiao::ErrorCode::InvalidCommand, message};
}

bool matches_register_reply(const damiao::CanFrame& frame, std::uint16_t esc,
    std::uint8_t operation, std::uint8_t rid)
{
    return frame.length == 8 && frame.data[0] == esc && frame.data[1] == 0
        && frame.data[2] == operation && frame.data[3] == rid;
}

bool wait_quiet(damiao::ICanTransport& transport, damiao::Deadline deadline,
    damiao::Deadline& last_receive)
{
    while (damiao::SteadyClock::now() < deadline
        && damiao::SteadyClock::now() < last_receive + management_quiet_period)
    {
        const auto received = transport.receive(
            std::min(deadline, last_receive + management_quiet_period));
        if (received.value)
        {
            last_receive = received.value->received_at;
        }
        else if (received.status.code != damiao::ErrorCode::Timeout)
        {
            return false;
        }
    }
    return damiao::SteadyClock::now() < deadline;
}

damiao::Result<damiao::CanFrame> wait_register_reply(damiao::ICanTransport& transport,
    std::uint16_t esc, std::uint8_t rid, damiao::Deadline deadline,
    damiao::Deadline& last_receive)
{
    while (damiao::SteadyClock::now() < deadline)
    {
        const auto received = transport.receive(deadline);
        if (!received.value)
        {
            if (received.status.code == damiao::ErrorCode::Timeout)
            {
                break;
            }
            return received;
        }
        last_receive = received.value->received_at;
        if (matches_register_reply(*received.value, esc, 0x33, rid))
        {
            return received;
        }
    }
    return {{damiao::ErrorCode::Timeout, "Register reply not received."}, std::nullopt};
}

damiao::Result<damiao::RegisterValue> read_register(damiao::ICanTransport& transport,
    const damiao::MotorAddress& address, std::uint8_t rid, damiao::Deadline deadline,
    damiao::Deadline& last_receive)
{
    if (!wait_quiet(transport, deadline, last_receive))
    {
        return {{damiao::ErrorCode::Timeout, "Quiet window expired."}, std::nullopt};
    }
    const auto encoded = damiao::DamiaoProtocol::encode_read_register(address.esc_id, rid);
    if (!encoded.value)
    {
        return {encoded.status, std::nullopt};
    }
    const auto sent = transport.send(*encoded.value);
    if (sent.code != damiao::ErrorCode::Ok)
    {
        return {sent, std::nullopt};
    }
    const auto reply = wait_register_reply(transport, address.esc_id, rid, deadline, last_receive);
    if (!reply.value)
    {
        return {reply.status, std::nullopt};
    }
    damiao::RegisterReplyExpectation expected;
    expected.address = address;
    expected.operation = damiao::RegisterOperation::Read;
    expected.register_id = rid;
    return damiao::DamiaoProtocol::decode_register_reply(*reply.value, expected);
}

bool address_seen(const std::vector<DiscoveredMotor>& motors, std::uint16_t esc, std::uint16_t mst)
{
    for (const auto& motor : motors)
    {
        if (motor.esc_id == esc || motor.mst_id == mst)
        {
            return true;
        }
    }
    return false;
}

damiao::Result<damiao::MotorState> query_state_scan(damiao::ICanTransport& transport,
    const damiao::MotorAddress& address, const damiao::MappingLimits& mapping,
    damiao::Deadline deadline, damiao::Deadline& last_receive)
{
    if (!wait_quiet(transport, deadline, last_receive))
    {
        return {{damiao::ErrorCode::Timeout, "Quiet window expired."}, std::nullopt};
    }
    const auto encoded = damiao::DamiaoProtocol::encode_state_query(address.esc_id);
    if (!encoded.value)
    {
        return {encoded.status, std::nullopt};
    }
    const auto sent = transport.send(*encoded.value);
    if (sent.code != damiao::ErrorCode::Ok)
    {
        return {sent, std::nullopt};
    }
    while (damiao::SteadyClock::now() < deadline)
    {
        const auto received = transport.receive(deadline);
        if (!received.value)
        {
            if (received.status.code == damiao::ErrorCode::Timeout)
            {
                break;
            }
            return {received.status, std::nullopt};
        }
        last_receive = received.value->received_at;
        const auto decoded = damiao::DamiaoProtocol::decode_feedback(
            *received.value, address, mapping);
        if (decoded.value)
        {
            return decoded;
        }
    }
    return {{damiao::ErrorCode::Timeout, "State query feedback not received."}, std::nullopt};
}

}  // namespace

const char* control_mode_name(damiao::ControlMode mode) noexcept
{
    switch (mode)
    {
        case damiao::ControlMode::Mit: return "Mit";
        case damiao::ControlMode::PositionVelocity: return "PositionVelocity";
        case damiao::ControlMode::Velocity: return "Velocity";
        case damiao::ControlMode::PositionCurrentLimit: return "PositionCurrentLimit";
    }
    return "Unknown";
}

ScanResult scan_motors(const ToolConfig& config, std::unique_ptr<damiao::ICanTransport> transport)
{
    ScanResult result;
    if (transport == nullptr)
    {
        result.status = invalid_scan("Transport is null.");
        return result;
    }

    damiao::TransportConfig transport_config;
    transport_config.interface_name = config.can_interface;
    const auto opened = transport->open(transport_config);
    if (opened.code != damiao::ErrorCode::Ok)
    {
        result.status = opened;
        return result;
    }

    auto last_receive = damiao::SteadyClock::now();
    std::size_t operable_count = 0;

    for (std::uint16_t esc = config.scan_esc_min; esc <= config.scan_esc_max; ++esc)
    {
        const auto deadline = damiao::SteadyClock::now()
            + std::chrono::milliseconds(config.scan_timeout_ms);
        if (!wait_quiet(*transport, deadline, last_receive))
        {
            continue;
        }
        const auto encoded = damiao::DamiaoProtocol::encode_read_register(esc, 0x08);
        if (!encoded.value)
        {
            continue;
        }
        const auto sent = transport->send(*encoded.value);
        if (sent.code != damiao::ErrorCode::Ok)
        {
            result.status = sent;
            transport->close();
            return result;
        }
        const auto probe = wait_register_reply(*transport, esc, 0x08, deadline, last_receive);
        if (!probe.value)
        {
            continue;
        }
        const damiao::MotorAddress address{esc, probe.value->id};
        if (damiao::DamiaoProtocol::validate_address(address).code != damiao::ErrorCode::Ok)
        {
            continue;
        }
        if (address_seen(result.motors, esc, address.mst_id))
        {
            continue;
        }

        DiscoveredMotor motor;
        motor.esc_id = esc;
        motor.mst_id = address.mst_id;

        constexpr std::uint8_t register_ids[] = {0x07, 0x0A, 0x15, 0x16, 0x17, 0x0E};
        bool register_ok = true;
        std::array<damiao::RegisterValue, 6> values;
        for (std::size_t index = 0; index < values.size(); ++index)
        {
            const auto read = read_register(*transport, address, register_ids[index],
                deadline, last_receive);
            if (read.status.code != damiao::ErrorCode::Ok || !read.value)
            {
                register_ok = false;
                break;
            }
            values[index] = *read.value;
        }
        if (!register_ok)
        {
            continue;
        }

        const auto mode_code = std::get<std::uint32_t>(values[1]);
        if (mode_code < 1 || mode_code > 4)
        {
            continue;
        }
        motor.mode = static_cast<damiao::ControlMode>(mode_code);
        motor.pmax_rad = std::get<float>(values[2]);
        motor.vmax_rad_s = std::get<float>(values[3]);
        motor.tmax_nm = std::get<float>(values[4]);
        motor.firmware_version = std::get<std::uint32_t>(values[5]);

        const damiao::MappingLimits mapping{motor.pmax_rad, motor.vmax_rad_s, motor.tmax_nm};
        if (damiao::DamiaoProtocol::validate_mapping_limits(mapping).code != damiao::ErrorCode::Ok)
        {
            continue;
        }

        const auto state = query_state_scan(*transport, address, mapping, deadline, last_receive);
        if (state.value)
        {
            motor.raw_status = state.value->raw_status;
            motor.output_position_rad = state.value->output_position_rad;
        }

        if (motor.mode != damiao::ControlMode::PositionVelocity)
        {
            motor.operable = false;
            motor.inoperable_reason = "非位置速度模式";
        }
        else if (operable_count >= damiao::max_motors)
        {
            motor.operable = false;
            motor.inoperable_reason = "超过单总线注册上限";
        }
        else
        {
            motor.operable = true;
            ++operable_count;
        }
        result.motors.push_back(motor);
    }

    transport->close();
    result.status = {};
    return result;
}

}  // namespace damiao_tools
