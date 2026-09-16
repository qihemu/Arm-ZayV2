#include <damiao_core/bus.hpp>
#include <damiao_core/registers.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <system_error>

namespace damiao
{
namespace
{

template<typename T>
// 构造不含结果值的统一失败返回对象。
Result<T> failure(ErrorCode code, const char* message)
{
    return {{code, message}, std::nullopt};
}

// 检查一台电机是否具备可用于发送门控的完整有限运动限制。
bool valid_motion_limits(const MotorConfig& motor)
{
    return motor.min_output_position_rad && motor.max_output_position_rad && motor.max_output_speed_rad_s
        && std::isfinite(*motor.min_output_position_rad) && std::isfinite(*motor.max_output_position_rad)
        && std::isfinite(*motor.max_output_speed_rad_s) && *motor.max_output_speed_rad_s > 0.0
        && *motor.min_output_position_rad < *motor.max_output_position_rad;
}

// 识别可能属于参数响应的帧，防止与相同 MST_ID 的普通反馈混淆。
bool parameter_shaped(const CanFrame& frame, std::uint16_t esc)
{
    if (frame.length < 4 || frame.data[0] != esc || frame.data[1] != 0)
    {
        return false;
    }
    return (frame.data[2] == 0xAA && frame.data[3] == 1)
        || ((frame.data[2] == 0x33 || frame.data[2] == 0x55) && register_info(frame.data[3]) != nullptr);
}

}  // namespace

DamiaoBus::DamiaoBus(std::unique_ptr<ICanTransport> transport) : transport_(std::move(transport))
{
    if (!transport_)
    {
        throw std::invalid_argument("DamiaoBus requires a transport.");
    }
    esc_index_.fill(-1);
    mst_index_.fill(-1);
}

DamiaoBus::~DamiaoBus()
{
    // 析构仅回收资源；停车和支撑交接必须由上层在析构之前完成。
    shutdown_resources();
}

Result<MotorIndex> DamiaoBus::register_motor(const MotorConfig& config)
{
    std::lock_guard<std::mutex> operation(operation_mutex_);
    std::lock_guard<std::mutex> cache(cache_mutex_);
    const auto mode = static_cast<std::uint32_t>(config.mode);
    if (state_ != BusState::Closed || motor_count_ == max_motors || config.name.empty()
        || DamiaoProtocol::validate_address(config.address).code != ErrorCode::Ok || mode < 1 || mode > 4
        || (config.mapping_confirmed && DamiaoProtocol::validate_mapping_limits(config.mapping).code != ErrorCode::Ok))
    {
        return failure<MotorIndex>(ErrorCode::InvalidConfiguration, "Invalid motor or registration state.");
    }
    // 检查所有模式 ID 的低八位与反馈 ID，避免反馈被另一电机当成目标。
    if (esc_index_[config.address.esc_id] >= 0 || mst_index_[config.address.mst_id] >= 0)
    {
        return failure<MotorIndex>(ErrorCode::InvalidConfiguration, "Duplicate ESC_ID or MST_ID.");
    }
    for (std::size_t index = 0; index < motor_count_; ++index)
    {
        const auto& existing = motors_[index];
        if (existing.name == config.name || (existing.address.mst_id & 0xFF) == config.address.esc_id
            || (config.address.mst_id & 0xFF) == existing.address.esc_id)
        {
            return failure<MotorIndex>(ErrorCode::InvalidConfiguration, "Motor name or low-byte ID conflict.");
        }
    }
    const auto index = motor_count_;
    motors_[index] = config;
    ++motor_count_;
    // 主动连接不能仅凭配置声称模式已经读回。
    motors_[index].mode_confirmed = false;
    revisions_[index] = 1;
    esc_index_[config.address.esc_id] = static_cast<int>(index);
    mst_index_[config.address.mst_id] = static_cast<int>(index);
    return {{}, index};
}

Status DamiaoBus::open(const BusConfig& config)
{
    std::lock_guard<std::mutex> operation(operation_mutex_);
    {
        std::lock_guard<std::mutex> cache(cache_mutex_);
        if (state_ != BusState::Closed || motor_count_ == 0 || config.feedback_timeout.count() <= 0
            || config.management_quiet_period.count() <= 0)
        {
            return {ErrorCode::InvalidConfiguration, "Register motors and provide positive bus timeouts before open."};
        }
    }
    const auto status = transport_->open(config.transport);
    if (status.code != ErrorCode::Ok)
    {
        return status;
    }
    {
        std::lock_guard<std::mutex> cache(cache_mutex_);
        // 每次主动打开都撤销配置可信标记，必须从本次设备会话重新读回。
        config_ = config;
        pending_ = {};
        diagnostics_ = {};
        last_receive_ = SteadyClock::now();
        for (std::size_t index = 0; index < motor_count_; ++index)
        {
            states_[index].valid = false;
            valid_after_[index] = SteadyClock::now();
            motors_[index].mode_confirmed = false;
            if (!config.transport.passive)
            {
                motors_[index].mapping_confirmed = false;
            }
        }
        state_ = BusState::Maintenance;
    }
    stop_ = false;
    try
    {
        receiver_ = std::thread(&DamiaoBus::receive_loop, this);
    }
    catch (const std::system_error& error)
    {
        shutdown_resources();
        return {ErrorCode::Disconnected, error.what()};
    }
    return {};
}

Status DamiaoBus::shutdown_resources()
{
    // 接收采用 10ms 有界轮询；先退出线程，再关闭 Socket，避免 fd 复用竞态。
    stop_ = true;
    received_.notify_all();
    if (receiver_.joinable())
    {
        receiver_.join();
    }
    const auto status = transport_->close();
    std::lock_guard<std::mutex> cache(cache_mutex_);
    state_ = BusState::Closed;
    pending_ = {};
    for (std::size_t index = 0; index < motor_count_; ++index)
    {
        states_[index].valid = false;
    }
    return status;
}

Status DamiaoBus::close()
{
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock())
    {
        return {ErrorCode::WouldBlock, "Another bus operation is active."};
    }
    {
        std::lock_guard<std::mutex> cache(cache_mutex_);
        if (state_ == BusState::Control)
        {
            return {ErrorCode::InvalidCommand, "Revoke control and complete the upper-layer stop policy before close."};
        }
    }
    return shutdown_resources();
}

BusState DamiaoBus::state() const
{
    std::lock_guard<std::mutex> cache(cache_mutex_);
    return state_;
}

BusDiagnostics DamiaoBus::diagnostics() const
{
    std::lock_guard<std::mutex> cache(cache_mutex_);
    return diagnostics_;
}

Result<MotorConfig> DamiaoBus::motor_config(MotorIndex index) const
{
    std::lock_guard<std::mutex> cache(cache_mutex_);
    if (index >= motor_count_)
    {
        return failure<MotorConfig>(ErrorCode::InvalidConfiguration, "Unknown motor index.");
    }
    return {{}, motors_[index]};
}

// 调用方持有 cache_mutex_，每轴用自身接收时间判定，不能由其他轴刷新。
ErrorCode DamiaoBus::feedback_code(MotorIndex index, Deadline now) const
{
    if (state_ == BusState::Closed)
    {
        return ErrorCode::Disconnected;
    }
    const auto& feedback = states_[index];
    if (!feedback.valid || feedback.mapping_revision != revisions_[index]
        || feedback.received_at < valid_after_[index] || now < feedback.received_at || now - feedback.received_at > config_.feedback_timeout)
    {
        return ErrorCode::StaleFeedback;
    }
    return feedback.raw_status > 1 ? ErrorCode::MotorFault : ErrorCode::Ok;
}

Result<MotorState> DamiaoBus::snapshot(MotorIndex index) const
{
    std::lock_guard<std::mutex> cache(cache_mutex_);
    if (index >= motor_count_)
    {
        return failure<MotorState>(ErrorCode::InvalidConfiguration, "Unknown motor index.");
    }
    const auto code = feedback_code(index, SteadyClock::now());
    // 故障反馈仍携带原始状态；未收到或过期样本不作为成功状态返回。
    return {{code, code == ErrorCode::Ok ? "" : "Feedback is unavailable, stale or faulty."}, states_[index]};
}

ErrorCode DamiaoBus::snapshot_into(MotorState* output, std::size_t count) const
{
    std::unique_lock<std::mutex> cache(cache_mutex_, std::try_to_lock);
    if (!cache.owns_lock())
    {
        return ErrorCode::WouldBlock;
    }
    if (output == nullptr || count != motor_count_)
    {
        return ErrorCode::InvalidConfiguration;
    }
    ErrorCode result = ErrorCode::Ok;
    const auto now = SteadyClock::now();
    for (std::size_t index = 0; index < count; ++index)
    {
        output[index] = states_[index];
        const auto code = feedback_code(index, now);
        if (code != ErrorCode::Ok)
        {
            output[index].valid = false;
            result = code;
        }
    }
    return result;
}

Status DamiaoBus::management_gate(MotorIndex index, bool require_disabled) const
{
    std::lock_guard<std::mutex> cache(cache_mutex_);
    if (index >= motor_count_)
    {
        return {ErrorCode::InvalidConfiguration, "Unknown motor index."};
    }
    if (state_ != BusState::Maintenance || config_.transport.passive)
    {
        return {ErrorCode::InvalidCommand, "Parameter management requires active maintenance ownership."};
    }
    for (std::size_t motor = 0; motor < motor_count_; ++motor)
    {
        if ((states_[motor].valid && states_[motor].raw_status == 1)
            || (require_disabled && (feedback_code(motor, SteadyClock::now()) != ErrorCode::Ok
                || states_[motor].raw_status != 0)))
        {
            return {ErrorCode::InvalidCommand, "Management writes require all motors confirmed disabled."};
        }
    }
    return {};
}

Status DamiaoBus::transact(MotorIndex index, const CanFrame& request, std::uint8_t operation,
    std::uint8_t rid, Deadline deadline, std::optional<RegisterValue>& value, bool* sent)
{
    std::unique_lock<std::mutex> cache(cache_mutex_);
    if (index >= motor_count_ || state_ == BusState::Closed || config_.transport.passive
        || (state_ == BusState::Fault && operation != 0xCC))
    {
        return {ErrorCode::InvalidCommand, "Transaction not allowed in current bus state."};
    }
    // 等待已观测总线静默，避免把旧队列回应关联到新事务；绝不无限等候。
    // 无事务序号，静默间隔仍不能消除所有远端迟到与普通反馈重合歧义。
    while (!stop_ && SteadyClock::now() < deadline
        && SteadyClock::now() < last_receive_ + config_.management_quiet_period)
    {
        received_.wait_until(cache, std::min(deadline, last_receive_ + config_.management_quiet_period));
    }
    if (stop_)
    {
        return {ErrorCode::Disconnected, "Receiver unavailable; request not sent."};
    }
    if (SteadyClock::now() >= deadline)
    {
        return {ErrorCode::Timeout, "No quiet management window before deadline; request not sent."};
    }
    if (state_ == BusState::Fault && operation != 0xCC)
    {
        return {ErrorCode::BusError, "Bus fault before transaction send."};
    }
    pending_ = {};
    pending_.index = index;
    pending_.operation = operation;
    pending_.rid = rid;
    pending_.active = true;
    pending_.sent_at = SteadyClock::now();
    pending_.deadline = deadline;
    cache.unlock();
    // 不持有缓存锁等待 send，发送与等待回应分离，接收线程始终能运行。
    const auto send_status = transport_->send(request);
    cache.lock();
    if (send_status.code != ErrorCode::Ok)
    {
        pending_.active = false;
        ++diagnostics_.send_failures;
        diagnostics_.last_error = send_status.code;
        state_ = BusState::Fault;
        return send_status;
    }
    if (sent != nullptr)
    {
        *sent = true;
    }
    received_.wait_until(cache, deadline, [this]
    {
        return pending_.done || stop_;
    });
    // 等待结束时只接受接收线程写入的匹配结果；未匹配响应一律视为超时未知。
    const auto code = pending_.done ? pending_.code : ErrorCode::Timeout;
    value = pending_.value;
    pending_.active = false;
    if (code != ErrorCode::Ok)
    {
        // 发出后的未知结果锁存 Fault，禁止相同会话自动重试或继续运动。
        diagnostics_.last_error = code;
        if (code == ErrorCode::Timeout)
        {
            ++diagnostics_.transaction_timeouts;
        }
        state_ = BusState::Fault;
    }
    return {code, code == ErrorCode::Ok ? "" : "Transaction failed or uncertain; no automatic retry."};
}

Result<RegisterValue> DamiaoBus::read_unlocked(MotorIndex index, std::uint8_t rid, Deadline deadline)
{
    const auto gate = management_gate(index, false);
    if (gate.code != ErrorCode::Ok)
    {
        return {gate, std::nullopt};
    }
    const auto config = motor_config(index);
    const auto encoded = DamiaoProtocol::encode_read_register(config.value->address.esc_id, rid);
    if (!encoded.value)
    {
        return {encoded.status, std::nullopt};
    }
    std::optional<RegisterValue> value;
    const auto status = transact(index, *encoded.value, 0x33, rid, deadline, value);
    return {status, value};
}

Result<RegisterValue> DamiaoBus::read_parameter(MotorIndex index, std::uint8_t rid, Deadline deadline)
{
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock())
    {
        return {{ErrorCode::WouldBlock, "Another management operation is active."}, std::nullopt};
    }
    return read_unlocked(index, rid, deadline);
}

void DamiaoBus::invalidate(MotorIndex index)
{
    ++revisions_[index];
    valid_after_[index] = SteadyClock::now();
    states_[index].valid = false;
}

Status DamiaoBus::synchronize_motor(MotorIndex index, Deadline deadline)
{
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock())
    {
        return {ErrorCode::WouldBlock, "Another management operation is active."};
    }
    const auto gate = management_gate(index, false);
    if (gate.code != ErrorCode::Ok)
    {
        return gate;
    }
    const auto previous = motor_config(index).value.value();
    {
        std::lock_guard<std::mutex> cache(cache_mutex_);
        invalidate(index);
        motors_[index].mapping_confirmed = false;
        motors_[index].mode_confirmed = false;
    }
    // 逐项查询使用同一总 deadline；全部校验通过后才发布可信映射与模式。
    std::array<RegisterValue, 7> values;
    constexpr std::uint8_t registers[] = {0x08, 0x07, 0x0A, 0x15, 0x16, 0x17, 0x0E};
    for (std::size_t item = 0; item < values.size(); ++item)
    {
        const auto result = read_unlocked(index, registers[item], deadline);
        if (result.status.code != ErrorCode::Ok || !result.value)
        {
            return result.status;
        }
        values[item] = *result.value;
    }
    const auto mode = std::get<std::uint32_t>(values[2]);
    MappingLimits mapping{std::get<float>(values[3]), std::get<float>(values[4]), std::get<float>(values[5])};
    if (std::get<std::uint32_t>(values[0]) != previous.address.esc_id
        || std::get<std::uint32_t>(values[1]) != previous.address.mst_id || mode < 1 || mode > 4
        || DamiaoProtocol::validate_mapping_limits(mapping).code != ErrorCode::Ok)
    {
        std::lock_guard<std::mutex> cache(cache_mutex_);
        state_ = BusState::Fault;
        return {ErrorCode::InvalidConfiguration, "Device address, mode or mapping does not match configuration."};
    }
    std::lock_guard<std::mutex> cache(cache_mutex_);
    motors_[index].mode = static_cast<ControlMode>(mode);
    motors_[index].mode_confirmed = true;
    motors_[index].mapping = mapping;
    motors_[index].mapping_confirmed = true;
    motors_[index].firmware_version = std::get<std::uint32_t>(values[6]);
    // 提交时建立生效时间，拒绝按新映射解码旧 Socket 队列中的反馈。
    valid_after_[index] = SteadyClock::now();
    states_[index].valid = false;
    return {};
}

Result<MotorState> DamiaoBus::query_unlocked(MotorIndex index, Deadline deadline)
{
    {
        std::lock_guard<std::mutex> cache(cache_mutex_);
        if (state_ == BusState::Control)
        {
            return failure<MotorState>(ErrorCode::InvalidCommand, "Active control cannot interleave management queries.");
        }
    }
    const auto config = motor_config(index);
    if (!config.value || !config.value->mapping_confirmed)
    {
        return failure<MotorState>(ErrorCode::InvalidConfiguration, "Query requires a confirmed mapping.");
    }
    const auto encoded = DamiaoProtocol::encode_state_query(config.value->address.esc_id);
    if (!encoded.value)
    {
        return {encoded.status, std::nullopt};
    }
    std::optional<RegisterValue> unused;
    const auto status = transact(index, *encoded.value, 0xCC, 0, deadline, unused);
    if (status.code != ErrorCode::Ok)
    {
        return {status, std::nullopt};
    }
    return snapshot(index);
}

Result<MotorState> DamiaoBus::query_state(MotorIndex index, Deadline deadline)
{
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock())
    {
        return {{ErrorCode::WouldBlock, "Another management operation is active."}, std::nullopt};
    }
    return query_unlocked(index, deadline);
}

ParameterWriteReport DamiaoBus::write_unlocked(MotorIndex index, std::uint8_t rid,
    const RegisterValue& value, Deadline deadline)
{
    ParameterWriteReport report;
    report.requested = value;
    report.status = management_gate(index, true);
    if (report.status.code != ErrorCode::Ok)
    {
        return report;
    }
    // 在线地址与波特率变更需要固件确认后的迁移流程；本版拒绝，避免失联盲发。
    if (rid == 0x07 || rid == 0x08 || rid == 0x23)
    {
        report.status = {ErrorCode::Unsupported, "Online ID/bitrate migration is not enabled."};
        return report;
    }
    const auto config = motor_config(index).value.value();
    const auto encoded = DamiaoProtocol::encode_write_register(config.address.esc_id, rid, value);
    if (!encoded.value)
    {
        report.status = encoded.status;
        return report;
    }
    const auto original = read_unlocked(index, rid, deadline);
    report.status = original.status;
    report.previous = original.value;
    if (original.status.code != ErrorCode::Ok)
    {
        return report;
    }
    // 写入前保留旧值；从发出写请求起先撤销相关配置和反馈的可信状态。
    const bool mapping_change = rid >= 0x15 && rid <= 0x17;
    {
        std::lock_guard<std::mutex> cache(cache_mutex_);
        invalidate(index);
        if (mapping_change)
        {
            motors_[index].mapping_confirmed = false;
        }
        if (rid == 0x0A)
        {
            motors_[index].mode_confirmed = false;
        }
    }
    std::optional<RegisterValue> acknowledgement;
    report.status = transact(index, *encoded.value, 0x55, rid, deadline, acknowledgement, &report.write_sent);
    if (report.status.code != ErrorCode::Ok)
    {
        return report;
    }
    // 设备确认和再次读取必须同时等于请求值，避免把迟到或错误响应当作成功。
    const auto actual = read_unlocked(index, rid, deadline);
    report.readback = actual.value;
    report.status = actual.status;
    if (actual.status.code != ErrorCode::Ok)
    {
        return report;
    }
    if (acknowledgement != std::optional<RegisterValue>{value} || actual.value != std::optional<RegisterValue>{value})
    {
        std::lock_guard<std::mutex> cache(cache_mutex_);
        state_ = BusState::Fault;
        report.status = {ErrorCode::AmbiguousReply, "Write acknowledgement/readback differs; device result is uncertain."};
        return report;
    }
    // 只有完整验证成功后才一次性发布新的本地配置语义。
    std::lock_guard<std::mutex> cache(cache_mutex_);
    if (mapping_change)
    {
        if (rid == 0x15)
        {
            motors_[index].mapping.position_rad = std::get<float>(value);
        }
        else if (rid == 0x16)
        {
            motors_[index].mapping.velocity_rad_s = std::get<float>(value);
        }
        else
        {
            motors_[index].mapping.torque_nm = std::get<float>(value);
        }
        motors_[index].mapping_confirmed = config.mapping_confirmed;
    }
    if (rid == 0x0A)
    {
        motors_[index].mode = static_cast<ControlMode>(std::get<std::uint32_t>(value));
        motors_[index].mode_confirmed = true;
    }
    valid_after_[index] = SteadyClock::now();
    states_[index].valid = false;
    report.verified = true;
    report.configuration_revision = revisions_[index];
    return report;
}

ParameterWriteReport DamiaoBus::write_parameter_verified(MotorIndex index, std::uint8_t rid,
    const RegisterValue& value, Deadline deadline)
{
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock())
    {
        ParameterWriteReport report;
        report.requested = value;
        report.status = {ErrorCode::WouldBlock, "Another management operation is active."};
        return report;
    }
    return write_unlocked(index, rid, value, deadline);
}

Status DamiaoBus::switch_mode(MotorIndex index, ControlMode mode, Deadline deadline)
{
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock())
    {
        return {ErrorCode::WouldBlock, "Another management operation is active."};
    }
    return write_unlocked(index, 0x0A, std::uint32_t(mode), deadline).status;
}

Status DamiaoBus::command_unlocked(MotorIndex index, ManagementCommand command, Deadline deadline)
{
    auto config = motor_config(index);
    if (!config.value)
    {
        return config.status;
    }
    {
        std::lock_guard<std::mutex> cache(cache_mutex_);
        if (state_ == BusState::Closed || config_.transport.passive
            || (state_ == BusState::Fault && command != ManagementCommand::Disable))
        {
            return {ErrorCode::InvalidCommand, "Management command not allowed in current state."};
        }
        if (!config.value->mode_confirmed || !config.value->mapping_confirmed)
        {
            return {ErrorCode::InvalidConfiguration, "Management command requires synchronized mode and mapping."};
        }
        if (command == ManagementCommand::Enable && config.value->mode != ControlMode::PositionVelocity)
        {
            return {ErrorCode::Unsupported, "Only position-velocity enable is exposed in this version."};
        }
        if (command != ManagementCommand::Disable)
        {
            if (state_ != BusState::Maintenance || !states_[index].valid
                || SteadyClock::now() - states_[index].received_at > config_.feedback_timeout
                || states_[index].raw_status == 1
                || (command == ManagementCommand::Enable && states_[index].raw_status != 0))
            {
                return {ErrorCode::InvalidCommand, "Explicit command requires a fresh stopped maintenance state."};
            }
        }
    }
    const auto encoded = DamiaoProtocol::encode_management_command(config.value->address.esc_id,
        config.value->mode, command);
    if (!encoded.value)
    {
        return encoded.status;
    }
    if (SteadyClock::now() >= deadline)
    {
        return {ErrorCode::Timeout, "Command deadline expired; nothing sent."};
    }
    const auto sent = transport_->send(*encoded.value);
    if (sent.code != ErrorCode::Ok)
    {
        std::lock_guard<std::mutex> cache(cache_mutex_);
        ++diagnostics_.send_failures;
        diagnostics_.last_error = sent.code;
        state_ = BusState::Fault;
        return sent;
    }
    const auto feedback = query_unlocked(index, deadline);
    if (feedback.status.code != ErrorCode::Ok || !feedback.value)
    {
        std::lock_guard<std::mutex> cache(cache_mutex_);
        state_ = BusState::Fault;
        states_[index].valid = false;
        return feedback.status;
    }
    // 管理命令没有事务序号，随后主动查询状态作为最小执行确认。
    const auto expected = command == ManagementCommand::Enable ? 1 : 0;
    if (feedback.value->raw_status != expected)
    {
        std::lock_guard<std::mutex> cache(cache_mutex_);
        state_ = BusState::Fault;
        return {ErrorCode::MotorFault, "Motor did not confirm requested enabled/disabled state."};
    }
    if (command == ManagementCommand::SaveZero)
    {
        const bool near_zero = std::abs(feedback.value->output_position_rad)
            <= 2.0 * config.value->mapping.position_rad / 65535.0;
        std::lock_guard<std::mutex> cache(cache_mutex_);
        invalidate(index);
        if (!near_zero)
        {
            state_ = BusState::Fault;
            return {ErrorCode::AmbiguousReply, "Zero command result was not confirmed."};
        }
    }
    return {};
}

Status DamiaoBus::enable(MotorIndex index, Deadline deadline)
{
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock())
    {
        return {ErrorCode::WouldBlock, "Another management operation is active."};
    }
    return command_unlocked(index, ManagementCommand::Enable, deadline);
}

Status DamiaoBus::disable(MotorIndex index, Deadline deadline)
{
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock())
    {
        return {ErrorCode::WouldBlock, "Another management operation is active."};
    }
    {
        std::lock_guard<std::mutex> cache(cache_mutex_);
        if (state_ == BusState::Control)
        {
            // 显式失能撤销整组新目标许可，停止策略仍由使用方负责。
            state_ = BusState::Maintenance;
        }
    }
    return command_unlocked(index, ManagementCommand::Disable, deadline);
}

Status DamiaoBus::clear_error(MotorIndex index, Deadline deadline)
{
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock())
    {
        return {ErrorCode::WouldBlock, "Another management operation is active."};
    }
    return command_unlocked(index, ManagementCommand::ClearError, deadline);
}

Status DamiaoBus::save_zero(MotorIndex index, Deadline deadline)
{
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock())
    {
        return {ErrorCode::WouldBlock, "Another management operation is active."};
    }
    const auto gate = management_gate(index, true);
    return gate.code == ErrorCode::Ok ? command_unlocked(index, ManagementCommand::SaveZero, deadline) : gate;
}

Status DamiaoBus::save_parameters(MotorIndex index, Deadline deadline)
{
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock())
    {
        return {ErrorCode::WouldBlock, "Another management operation is active."};
    }
    const auto gate = management_gate(index, true);
    if (gate.code != ErrorCode::Ok)
    {
        return gate;
    }
    if (deadline - SteadyClock::now() < std::chrono::milliseconds(30))
    {
        return {ErrorCode::InvalidCommand, "Flash save requires at least the documented 30ms time allowance."};
    }
    const auto config = motor_config(index).value.value();
    const auto encoded = DamiaoProtocol::encode_save_parameters(config.address.esc_id);
    std::optional<RegisterValue> unused;
    return transact(index, *encoded.value, 0xAA, 1, deadline, unused);
}

Status DamiaoBus::begin_control()
{
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock())
    {
        return {ErrorCode::WouldBlock, "Another bus operation is active."};
    }
    std::lock_guard<std::mutex> cache(cache_mutex_);
    if (state_ != BusState::Maintenance || config_.transport.passive)
    {
        return {ErrorCode::InvalidCommand, "Control requires active maintenance ownership."};
    }
    const auto now = SteadyClock::now();
    for (std::size_t index = 0; index < motor_count_; ++index)
    {
        const auto& config = motors_[index];
        const auto code = feedback_code(index, now);
        if (code != ErrorCode::Ok || states_[index].raw_status != 1
            || !config.mode_confirmed || !config.mapping_confirmed
            || config.mode != ControlMode::PositionVelocity || !valid_motion_limits(config)
            || states_[index].output_position_rad < *config.min_output_position_rad
            || states_[index].output_position_rad > *config.max_output_position_rad)
        {
            return {code == ErrorCode::Ok ? ErrorCode::InvalidConfiguration : code,
                "Control requires all motors enabled, fresh, synchronized, position-velocity and limited."};
        }
    }
    state_ = BusState::Control;
    return {};
}

Status DamiaoBus::end_control()
{
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock())
    {
        return {ErrorCode::WouldBlock, "Another bus operation is active."};
    }
    std::lock_guard<std::mutex> cache(cache_mutex_);
    if (state_ == BusState::Control)
    {
        state_ = BusState::Maintenance;
    }
    return state_ == BusState::Fault ? Status{ErrorCode::BusError, "Fault remains latched."} : Status{};
}

ErrorCode DamiaoBus::send_position_velocity_batch(const PositionVelocityCommand* commands,
    std::size_t count, ErrorCode* results)
{
    if (commands == nullptr || results == nullptr || count == 0 || count > max_motors)
    {
        return ErrorCode::InvalidCommand;
    }
    std::fill_n(results, count, ErrorCode::NotExecuted);
    std::unique_lock<std::mutex> operation(operation_mutex_, std::try_to_lock);
    if (!operation.owns_lock())
    {
        return ErrorCode::WouldBlock;
    }
    std::array<CanFrame, max_motors> frames;
    {
        std::unique_lock<std::mutex> cache(cache_mutex_, std::try_to_lock);
        if (!cache.owns_lock())
        {
            return ErrorCode::WouldBlock;
        }
        if (state_ != BusState::Control || count != motor_count_)
        {
            return ErrorCode::InvalidCommand;
        }
        const auto now = SteadyClock::now();
        // 全量校验通过后才编码和发送，不让前轴执行后轴的非法任务。
        for (std::size_t index = 0; index < count; ++index)
        {
            const auto& motor = motors_[index];
            const auto& command = commands[index];
            auto code = feedback_code(index, now);
            if (code == ErrorCode::Ok && states_[index].raw_status != 1)
            {
                code = ErrorCode::MotorFault;
            }
            if (code == ErrorCode::Ok && (!valid_motion_limits(motor)
                || !std::isfinite(command.output_position_rad) || !std::isfinite(command.max_output_speed_rad_s)
                || std::abs(command.output_position_rad) > std::numeric_limits<float>::max()
                || command.output_position_rad < *motor.min_output_position_rad
                || command.output_position_rad > *motor.max_output_position_rad
                || command.max_output_speed_rad_s < 0.0
                || command.max_output_speed_rad_s > *motor.max_output_speed_rad_s
                || command.max_output_speed_rad_s > std::numeric_limits<float>::max()
                || (command.output_position_rad != 0.0 && static_cast<float>(command.output_position_rad) == 0.0F)
                || (command.max_output_speed_rad_s != 0.0 && static_cast<float>(command.max_output_speed_rad_s) == 0.0F)))
            {
                code = ErrorCode::InvalidCommand;
            }
            if (code != ErrorCode::Ok)
            {
                results[index] = code;
                if (code == ErrorCode::StaleFeedback || code == ErrorCode::MotorFault)
                {
                    state_ = BusState::Fault;
                    diagnostics_.last_error = code;
                }
                return code;
            }
        }
        for (std::size_t index = 0; index < count; ++index)
        {
            frames[index] = *DamiaoProtocol::encode_position_velocity(motors_[index].address.esc_id,
                commands[index]).value;
        }
    }
    for (std::size_t index = 0; index < count; ++index)
    {
        // 发送阶段不重试；部分发送后失败必须锁存故障并由上层重新规划。
        results[index] = transport_->send_code(frames[index]);
        if (results[index] != ErrorCode::Ok)
        {
            std::lock_guard<std::mutex> cache(cache_mutex_);
            ++diagnostics_.send_failures;
            diagnostics_.last_error = results[index];
            state_ = BusState::Fault;
            return index == 0 ? results[index] : ErrorCode::PartialFailure;
        }
    }
    return ErrorCode::Ok;
}

void DamiaoBus::receive_loop()
{
    while (!stop_)
    {
        // 短截止时间让关闭流程可在有界时间内停止线程，同时保持单一接收者。
        const auto result = transport_->receive(SteadyClock::now() + std::chrono::milliseconds(10));
        if (result.status.code == ErrorCode::Ok && result.value)
        {
            route_frame(*result.value);
        }
        else if (result.status.code == ErrorCode::InvalidFrame)
        {
            std::lock_guard<std::mutex> cache(cache_mutex_);
            ++diagnostics_.received_frames;
            ++diagnostics_.rejected_frames;
            diagnostics_.last_error = ErrorCode::InvalidFrame;
        }
        else if (result.status.code != ErrorCode::Timeout && result.status.code != ErrorCode::WouldBlock
            && result.status.code != ErrorCode::InvalidFrame)
        {
            std::lock_guard<std::mutex> cache(cache_mutex_);
            state_ = BusState::Fault;
            diagnostics_.last_error = result.status.code;
            diagnostics_.transport_error_detail = result.status.message;
            if (pending_.active)
            {
                pending_.done = true;
                pending_.code = result.status.code;
            }
            stop_ = true;
            received_.notify_all();
            break;
        }
    }
}

void DamiaoBus::route_frame(const CanFrame& frame)
{
    std::lock_guard<std::mutex> cache(cache_mutex_);
    last_receive_ = SteadyClock::now();
    ++diagnostics_.received_frames;
    received_.notify_all();
    if (frame.id > 0x7FF || frame.length > 8 || mst_index_[frame.id] < 0)
    {
        ++diagnostics_.rejected_frames;
        return;
    }
    const auto index = static_cast<std::size_t>(mst_index_[frame.id]);
    const auto& motor = motors_[index];
    const bool parameter = parameter_shaped(frame, motor.address.esc_id);
    const bool matches_pending = pending_.active && pending_.index == index
        && frame.received_at >= pending_.sent_at && frame.received_at <= pending_.deadline;
    if (parameter)
    {
        // 参数形状至少同时核对 MST_ID、完整 ESC_ID、操作码、RID 和长度。
        // 没有匹配事务的形状报文不冒充普通状态；重合时宁可丢弃不确定样本。
        if (matches_pending && pending_.operation == frame.data[2] && pending_.rid == frame.data[3])
        {
            Status status;
            if (pending_.operation == 0xAA)
            {
                status = DamiaoProtocol::decode_save_reply(frame, motor.address);
            }
            else
            {
                const auto value = DamiaoProtocol::decode_register_reply(frame,
                    {motor.address, static_cast<RegisterOperation>(pending_.operation), pending_.rid});
                status = value.status;
                pending_.value = value.value;
            }
            pending_.done = true;
            pending_.code = status.code;
        }
        else if (matches_pending && pending_.operation == 0xCC)
        {
            pending_.done = true;
            pending_.code = ErrorCode::AmbiguousReply;
            ++diagnostics_.ambiguous_frames;
        }
        else
        {
            ++diagnostics_.ambiguous_frames;
        }
        return;
    }
    if (!motor.mapping_confirmed)
    {
        return;
    }
    const auto decoded = DamiaoProtocol::decode_feedback(frame, motor.address, motor.mapping, revisions_[index]);
    // 配置提交前、倒序到达或比当前缓存更旧的反馈都不能覆盖最新状态。
    if (!decoded.value || frame.received_at < valid_after_[index]
        || frame.received_at <= states_[index].received_at)
    {
        ++diagnostics_.rejected_frames;
        return;
    }
    const auto sequence = states_[index].sequence + 1;
    states_[index] = *decoded.value;
    states_[index].sequence = sequence;
    if (matches_pending && pending_.operation == 0xCC)
    {
        pending_.done = true;
        pending_.code = ErrorCode::Ok;
    }
    if (state_ == BusState::Control)
    {
        // 控制期间任何失能、设备故障或越过软件位置边界都会立即锁存总线故障。
        const bool position_invalid = !valid_motion_limits(motor)
            || states_[index].output_position_rad < *motor.min_output_position_rad
            || states_[index].output_position_rad > *motor.max_output_position_rad;
        if (states_[index].raw_status != 1 || position_invalid)
        {
            state_ = BusState::Fault;
            diagnostics_.last_error = states_[index].raw_status != 1 ? ErrorCode::MotorFault : ErrorCode::InvalidCommand;
        }
    }
}

}  // namespace damiao
