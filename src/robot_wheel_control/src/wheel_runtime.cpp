#include <robot_wheel_control/runtime.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <unistd.h>
namespace robot_wheel_control
{
using namespace damiao;
WheelRuntime::WheelRuntime(Configuration c, std::unique_ptr<ICanTransport> transport) : config_(std::move(c))
{
    if (!transport)
    {
        if (config_.backend == "direct_usb_sdk")
        {
            transport = std::make_unique<DmUsbCanTransport>(config_.sdk);
        }
        else if (config_.backend == "socketcan")
        {
            transport = std::make_unique<SocketCanTransport>();
        }
        else
        {
            throw std::runtime_error("Only real direct_usb_sdk/socketcan backends are available");
        }
    }
    bus_ = std::make_unique<DirectCanWheelBackend>(config_.bus, config_.transport, std::move(transport));
    state_.session =
        std::to_string(getpid()) + "-" + std::to_string(SteadyClock::now().time_since_epoch().count());
}
WheelRuntime::~WheelRuntime()
{
    shutdown();
}
void WheelRuntime::start()
{
    const auto result = bus_->open();
    if (result.code != ErrorCode::Ok)
    {
        throw std::runtime_error(result.message);
    }
    running_ = true;
    receiving_ = true;
    receiver_ = std::thread([this] { receive_loop(); });
    transmitter_ = std::thread([this] { transmit_loop(); });
    manager_ = std::thread([this] { management_loop(); });
}
void WheelRuntime::shutdown()
{
    if (!running_.exchange(false))
    {
        return;
    }
    changed_.notify_all();
    if (transmitter_.joinable())
    {
        transmitter_.join();
    }
    if (manager_.joinable())
    {
        manager_.join();
    }
    receiving_ = false;
    if (receiver_.joinable())
    {
        receiver_.join();
    }
    bus_->close();
}
RuntimeState WheelRuntime::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = state_;
    result.command_fresh =
        source_stamp_ != Deadline{} &&
        SteadyClock::now() - source_stamp_ <= std::chrono::milliseconds(config_.command_timeout_ms);
    return result;
}
void WheelRuntime::receive_loop()
{
    // shutdown期间TX停车仍需要RX应答：以TX是否退出为结束条件见shutdown流程。
    while (receiving_)
    {
        const auto s = bus_->receive_once(SteadyClock::now() + std::chrono::milliseconds(5));
        if (s.code != ErrorCode::Ok && s.code != ErrorCode::Timeout)
        {
            request_fault(s.message);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}
void WheelRuntime::request_fault(const std::string &reason)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.fault)
    {
        state_.fault = true;
        ++state_.fault_sequence;
        state_.reason = reason;
        fault_reason_ = reason;
        state_.permitted = false;
        state_.lifecycle = 6;
        stop_pending_ = true;
        ++stop_generation_;
    }
}
void WheelRuntime::fault(const std::string &reason)
{
    request_fault(reason);
    const auto stopped = bus_->stop_pair();
    update_measurements();
    std::lock_guard<std::mutex> lock(mutex_);
    state_.enabled = false;
    state_.permitted = false;
    state_.lifecycle = 6;
    if (stopped.code != ErrorCode::Ok)
    {
        state_.reason += "; disable unconfirmed: " + stopped.message;
    }
    sent_ = {};
    target_ = {};
    state_.target = {};
    stop_pending_ = false;
}
void WheelRuntime::authorize_source()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.enabled && state_.permitted && !state_.fault)
    {
        source_stamp_ = SteadyClock::now();
    }
}
bool WheelRuntime::command(const std::array<double, 2> &speed, bool controller_write)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.enabled || !state_.permitted || state_.fault || state_.relative_active)
    {
        return false;
    }
    for (double s : speed)
    {
        if (!std::isfinite(s) || std::abs(s) > config_.wheel_speed + 1e-9)
        {
            return false;
        }
    }
    if (controller_write && config_.mode != "base")
    {
        return false;
    }
    if (!controller_write && config_.mode != "bench")
    {
        return false;
    }
    target_ = speed;
    command_stamp_ = SteadyClock::now();
    if (!controller_write)
    {
        source_stamp_ = command_stamp_;
    }
    return true;
}
bool WheelRuntime::submit(Operation op, const std::string &session, const std::string &id,
                          std::uint64_t expected, bool disable, std::string &reason, const RelativeGoal &goal)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (id.empty() || id.size() > 128)
    {
        reason = "request_id must contain 1..128 bytes";
        return false;
    }
    if (op != Operation::Stop && session != state_.session)
    {
        reason = "Session mismatch";
        return false;
    }
    std::ostringstream payload;
    payload << std::setprecision(17) << int(op) << ':' << disable << ':' << expected << ':' << int(goal.kind)
            << ':' << goal.value << ':' << goal.max_speed << ':' << goal.timeout_s;
    const auto fingerprint = payload.str();
    const auto old = results_.find(id);
    if (old != results_.end())
    {
        if (fingerprints_[id] != fingerprint)
        {
            reason = "request_id reused with different payload";
            return false;
        }
        reason = "Existing request";
        return true;
    }
    if (op == Operation::Enable && (!state_.configured || state_.fault))
    {
        reason = "Not configured or fault latched: " + state_.reason;
        return false;
    }
    if ((op == Operation::Enable || op == Operation::Relative) && state_.relative_active)
    {
        reason = "Relative motion owns wheel commands; stop it first";
        return false;
    }
    if (op == Operation::Relative)
    {
        // 全部请求值在使能之前验证；默认不允许盲目切换现有手动控制来源。
        if (!state_.configured || state_.fault || state_.enabled || state_.lifecycle != 1 ||
            !requests_.empty() || tx_job_ || stop_pending_)
        {
            reason = "Relative motion requires configured, disabled, fault-free and idle state";
            return false;
        }
        if (!std::isfinite(goal.value) || !std::isfinite(goal.max_speed) || !std::isfinite(goal.timeout_s) ||
            goal.max_speed <= 0 || goal.timeout_s <= 0 || goal.timeout_s > config_.relative.max_timeout_s ||
            goal.kind < 1 || goal.kind > 3 || std::abs(goal.value) < 1e-6)
        {
            reason = "Invalid relative motion value, speed or timeout";
            return false;
        }
        if (config_.mode == "base" || (goal.kind == 3 && config_.mode != "bench") ||
            (goal.kind != 3 && (config_.radius[0] <= 0 || config_.radius[1] <= 0 || config_.separation <= 0)))
        {
            reason = "Use standalone bench/relative runner; distance/yaw require wheel radii and "
                     "tread-centre separation";
            return false;
        }
        const double limit = goal.kind == 1
                                 ? config_.relative.max_distance
                                 : (goal.kind == 2 ? config_.relative.max_yaw : config_.bench_travel * 0.8);
        if (std::abs(goal.value) > limit || (goal.kind == 3 && goal.max_speed > config_.wheel_speed))
        {
            reason = "Relative target or wheel speed exceeds configured limit";
            return false;
        }
    }
    if (op == Operation::Clear && (!state_.fault || expected != state_.fault_sequence))
    {
        reason = "Fault sequence mismatch";
        return false;
    }
    if (op != Operation::Stop && requests_.size() >= std::size_t(config_.management_capacity))
    {
        reason = "Management queue full";
        return false;
    }
    Request request{op, id, disable, stop_generation_, goal};
    if (op == Operation::Stop || op == Operation::Disable)
    {
        ++stop_generation_;
        request.generation = stop_generation_;
        state_.permitted = false;
        target_ = {};
        source_stamp_ = {};
        command_stamp_ = {};
        stop_pending_ = true;
    }
    results_[id] = {id, "Queued", 1};
    fingerprints_[id] = fingerprint;
    state_.last_request = id;
    state_.last_result = 1;
    if (op == Operation::Relative)
    {
        state_.relative_active = true;
        state_.relative_id = id;
        state_.relative_goal = goal;
        state_.relative_target = {};
        state_.relative_travel = {};
        state_.relative_measured = 0;
        heartbeat_stamp_ = SteadyClock::now();
    }
    if (op == Operation::Stop)
    {
        if (stop_request_)
        {
            auto &previous = results_[stop_request_->id];
            previous.status = 4;
            previous.reason = "Superseded by newer stop";
            events_.push_back(previous);
            completed_.push_back(previous.id);
        }
        stop_request_ = request;
    }
    else
    {
        requests_.push_back(request);
    }
    prune_history_locked();
    changed_.notify_all();
    reason = "Accepted asynchronously";
    return true;
}
OperationResult WheelRuntime::result(const std::string &id) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = results_.find(id);
    return it == results_.end() ? OperationResult{id, "Unknown/expired request", 0} : it->second;
}
std::vector<OperationResult> WheelRuntime::take_events()
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto events = std::move(events_);
    events_.clear();
    return events;
}
void WheelRuntime::finish(const Request &r, const Status &s)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto &result = results_[r.id];
    result.status = s.code == ErrorCode::Ok ? 2 : (s.code == ErrorCode::NotExecuted ? 4 : 3);
    result.reason = s.message.empty() ? "Completed" : s.message;
    events_.push_back(result);
    completed_.push_back(r.id);
    state_.last_request = r.id;
    state_.last_result = result.status;
    if (r.operation == Operation::Relative && state_.relative_id == r.id)
    {
        state_.relative_active = false;
    }
    prune_history_locked();
}
void WheelRuntime::prune_history_locked()
{
    // 已完成及被新Stop替换的结果均受同一容量限制；不驱逐活动事务。
    while (completed_.size() > std::size_t(config_.history_capacity))
    {
        results_.erase(completed_.front());
        fingerprints_.erase(completed_.front());
        completed_.pop_front();
    }
    if (events_.size() > std::size_t(config_.history_capacity))
    {
        events_.erase(events_.begin());
    }
}
void WheelRuntime::management_loop()
{
    while (running_)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait_for(lock, std::chrono::milliseconds(10),
                          [this] { return !running_ || stop_request_ || !requests_.empty(); });
        if (!running_)
        {
            break;
        }
        if (!stop_request_ && requests_.empty())
        {
            continue;
        }
        Request r;
        if (stop_request_)
        {
            r = *stop_request_;
            stop_request_.reset();
        }
        else
        {
            r = requests_.front();
            requests_.pop_front();
        }
        auto promise = std::make_shared<std::promise<Status>>();
        auto future = promise->get_future();
        tx_job_ = [this, r, promise]
        {
            try
            {
                promise->set_value(perform(r));
            }
            catch (const std::exception &e)
            {
                fault(e.what());
                promise->set_value({ErrorCode::BusError, e.what()});
            }
        };
        changed_.notify_all();
        lock.unlock();
        while (running_ && future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready)
        {
        }
        if (!running_)
        {
            break;
        }
        const auto outcome = future.get();
        // 相对运动的成功表示已开始，完成结果由TX闭环到位/中止后发布。
        if (r.operation != Operation::Relative || outcome.code != ErrorCode::Ok)
        {
            finish(r, outcome);
        }
    }
}
Status WheelRuntime::perform(const Request &r)
{
    if (r.operation == Operation::Relative)
    {
        return start_relative(r);
    }
    if (r.operation == Operation::Enable)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (r.generation != stop_generation_ || state_.fault || !state_.configured)
            {
                return {ErrorCode::NotExecuted, "Enable cancelled by stop/fault"};
            }
            state_.lifecycle = 2;
            state_.permitted = false;
            command_stamp_ = {};
            source_stamp_ = {};
            target_ = {};
        }
        // 待机容错不作为使能依据：必须先收到本次新鲜的双轮失能确认。
        if (!snapshot().enabled)
        {
            const auto ready = bus_->disable_pair();
            if (ready.code != ErrorCode::Ok)
            {
                fault(ready.message);
                return ready;
            }
            update_measurements();
            if (!trackers_[0].valid() || !trackers_[1].valid())
            {
                if (config_.mode != "bench")
                {
                    fault("Position continuity lost; restart/new odometry session required");
                    return {ErrorCode::StaleFeedback, "Base position continuity lost"};
                }
                // 悬空台架只重建本次小角度行程基准，不宣称恢复丢失区间的里程。
                for (auto &tracker : trackers_)
                {
                    tracker.reset();
                }
                update_measurements();
            }
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (r.generation != stop_generation_ || state_.fault)
            {
                return {ErrorCode::NotExecuted, "Enable cancelled during fresh feedback check"};
            }
        }
        const auto result = bus_->enable_pair();
        if (result.code != ErrorCode::Ok)
        {
            fault(result.message);
            return result;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (r.generation == stop_generation_ && !state_.fault)
            {
                state_.enabled = true;
                state_.permitted = true;
                state_.lifecycle = 3;
                state_.reason = "Enabled; waiting for a new command";
                enabled_since_ = SteadyClock::now();
                bench_origin_ = state_.position;
                sent_ = {};
                return {};
            }
        }
        bus_->stop_pair();
        return {ErrorCode::NotExecuted, "Enable cancelled by stop"};
    }
    if (r.operation == Operation::Clear)
    {
        // 对连续位置曾失效的实车不悄悄重置odom；恢复需要显式重启新会话。
        if (config_.mode == "base")
        {
            return {ErrorCode::NotExecuted, "Base fault recovery requires restart/new odometry session"};
        }
        const auto cleared = bus_->clear_pair();
        if (cleared.code != ErrorCode::Ok)
        {
            return cleared;
        }
        const auto verified = bus_->verify_configuration();
        if (verified.code != ErrorCode::Ok)
        {
            return verified;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        state_.fault = false;
        fault_reason_.clear();
        state_.configured = true;
        state_.enabled = false;
        state_.permitted = false;
        state_.lifecycle = 1;
        state_.reason = "Fault cleared; explicit enable required";
        for (auto &tracker : trackers_)
        {
            tracker.reset();
        }
        return {};
    }
    return controlled_stop(r.operation == Operation::Disable || r.disable_after);
}
Status WheelRuntime::controlled_stop(bool disable)
{
    // Stop/Disable由管理线程调度到唯一TX；取消任务后仍完成原有停车动作。
    std::optional<Request> cancelled;
    if (active_relative_)
    {
        cancelled = *active_relative_;
        active_relative_.reset();
    }
    // 析构顺序保证结果在停车完成且mutex_释放后发布，而非提前报告已取消。
    struct OnExit
    {
        std::function<void()> function;
        ~OnExit()
        {
            function();
        }
    } complete_cancel{[this, &cancelled]
                      {
                          if (cancelled)
                          {
                              finish(*cancelled, {ErrorCode::NotExecuted,
                                                  "Relative motion cancelled by stop; check stop result"});
                          }
                      }};
    std::uint64_t generation;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        generation = stop_generation_;
        state_.permitted = false;
        target_ = {};
        source_stamp_ = {};
        command_stamp_ = {};
        state_.lifecycle = 5;
    }
    // 管理任务在TX线程执行；RX仍并行。停止过程无新的非零目标来源。
    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(config_.stop_timeout_ms);
    auto now = SteadyClock::now();
    while (running_ && snapshot().enabled && now < deadline)
    {
        if (snapshot().fault)
        {
            disable = true;
            break;
        }
        for (auto &speed : sent_)
        {
            speed = std::clamp(0.0, speed - config_.wheel_acceleration / config_.control_hz,
                               speed + config_.wheel_acceleration / config_.control_hz);
        }
        std::array<double, 2> raw;
        for (std::size_t i = 0; i < 2; ++i)
        {
            raw[i] = sent_[i] * config_.direction[i] * config_.reduction[i];
        }
        const auto sent = bus_->send_velocity_pair(raw);
        if (sent.code != ErrorCode::Ok)
        {
            break;
        }
        update_measurements();
        if (std::abs(sent_[0]) < 1e-9 && std::abs(sent_[1]) < 1e-9 && snapshot().standstill)
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(1 / config_.control_hz));
        now = SteadyClock::now();
    }
    const auto stopping_state = snapshot();
    disable = disable || stopping_state.fault;
    const bool settled = stopping_state.standstill;
    Status stopped;
    if (disable || !settled)
    {
        stopped = bus_->stop_pair();
        update_measurements();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    state_.enabled = !(disable || !settled) && stopped.code == ErrorCode::Ok;
    state_.permitted = false;
    state_.target = {};
    sent_ = {};
    state_.lifecycle = state_.fault ? 6 : (state_.enabled ? 3 : 1);
    // 新到的故障/停止请求不能被上一个停车事务清掉。
    if (generation == stop_generation_)
    {
        stop_pending_ = false;
    }
    if (stopped.code != ErrorCode::Ok)
    {
        if (!state_.fault)
        {
            fault_reason_ = "Disable not confirmed: " + stopped.message;
            ++state_.fault_sequence;
        }
        state_.fault = true;
        state_.lifecycle = 6;
        state_.reason = fault_reason_;
        return stopped;
    }
    if (state_.fault)
    {
        state_.reason = fault_reason_;
        return {ErrorCode::Ok, "Drivers disabled; fault remains latched: " + fault_reason_};
    }
    state_.reason = settled ? "Standstill confirmed; motion authorization revoked"
                            : "Drivers disabled; physical standstill is not confirmed";
    return {ErrorCode::Ok, state_.reason};
}
void WheelRuntime::update_measurements()
{
    const auto raw = bus_->snapshot();
    const auto now = SteadyClock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    state_.motors = raw;
    state_.position_valid = true;
    state_.standstill = true;
    double ds = 0;
    for (std::size_t i = 0; i < 2; ++i)
    {
        const double previous = state_.position[i];
        const auto &m = config_.bus.motors[i];
        const bool good = trackers_[i].update(raw[i], config_.continuous_verified ? config_.wrap_period : 0,
                                              m.maximum_speed, 2 * m.mapping.position_rad / 65535.0,
                                              config_.unwrap_gap_ms / 1000.0);
        state_.position[i] = trackers_[i].position() * config_.direction[i] / config_.reduction[i];
        state_.velocity[i] = raw[i].output_velocity_rad_s * config_.direction[i] / config_.reduction[i];
        const auto timeout = state_.enabled ? config_.bus.feedback_timeout
                                            : std::chrono::milliseconds(config_.idle_feedback_timeout_ms);
        const bool fresh = raw[i].valid && now - raw[i].received_at <= timeout;
        state_.position_valid = state_.position_valid && good && fresh && config_.continuous_verified;
        if (config_.radius[i] > 0)
        {
            ds += (state_.position[i] - previous) * config_.radius[i] / 2;
        }
        const bool stationary = good && fresh && config_.standstill_speed > 0 &&
                                config_.standstill_position > 0 &&
                                std::abs(state_.velocity[i]) <= config_.standstill_speed;
        if (!stationary || std::abs(state_.position[i] - stationary_anchor_[i]) > config_.standstill_position)
        {
            stationary_since_[i] = now;
            stationary_anchor_[i] = state_.position[i];
        }
        state_.standstill = state_.standstill && stationary &&
                            now - stationary_since_[i] >= std::chrono::milliseconds(config_.standstill_ms);
    }
    if (state_.position_valid)
    {
        state_.path += std::abs(ds);
    }
    ++state_.sequence;
}
void WheelRuntime::transmit_loop()
{
    auto verified = bus_->verify_configuration();
    if (verified.code != ErrorCode::Ok)
    {
        fault(verified.message);
    }
    else
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.configured = true;
        state_.lifecycle = 1;
        state_.reason = "Configured, disabled; explicit enable required";
    }
    auto tick = SteadyClock::now();
    auto next_idle_poll = tick;
    while (running_)
    {
        std::function<void()> job;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            job = std::move(tx_job_);
            tx_job_ = {};
        }
        if (job)
        {
            job();
            tick = SteadyClock::now();
        }
        if (stop_pending_)
        {
            const auto pending = snapshot();
            if (pending.fault)
            {
                fault(pending.reason);
            }
            // 普通Stop由专用管理槽执行其disable_after选项；submit已经撤销许可并清空目标，
            // 等待调度的一个周期内TX只会向零减速。故障则在此立即失能。
            tick = SteadyClock::now();
        }
        auto state = snapshot();
        if (state.fault)
        {
            if (active_relative_)
            {
                end_relative({ErrorCode::BusError, state.reason}, true);
            }
            // 故障锁存不冻结黑板；仅刷新失能反馈，不清错、不使能、不重放目标。
            if (SteadyClock::now() >= next_idle_poll)
            {
                bus_->poll_disabled_pair();
                next_idle_poll =
                    SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
                                             std::chrono::duration<double>(1 / config_.idle_poll_hz));
            }
            update_measurements();
            tick = SteadyClock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        const auto now = SteadyClock::now();
        if (state.enabled && now - tick > std::chrono::milliseconds(config_.lateness_ms))
        {
            fault("Control scheduling deadline missed");
            continue;
        }
        if (state.configured)
        {
            update_measurements();
            state = snapshot();
            std::string error;
            for (std::size_t i = 0; i < 2; ++i)
            {
                const auto &m = state.motors[i];
                const auto timeout = state.enabled
                                         ? config_.bus.feedback_timeout
                                         : std::chrono::milliseconds(config_.idle_feedback_timeout_ms);
                if (!m.valid || now - m.received_at > timeout)
                {
                    error = "Wheel feedback stale";
                }
                else if (m.raw_status > 1)
                {
                    error = h55::status_description(m.raw_status);
                }
                else if (state.enabled && m.raw_status != 1)
                {
                    error = "Unexpected driver disable";
                }
                else if (!state.enabled && m.raw_status != 0)
                {
                    error = "Unexpected driver enable while disabled";
                }
                else if (std::abs(m.reported_torque_nm) > config_.torque_limit ||
                         m.mos_temperature_c > config_.driver_temperature ||
                         m.rotor_temperature_c > config_.motor_temperature ||
                         std::abs(m.output_velocity_rad_s) > config_.bus.motors[i].maximum_speed)
                {
                    error = "Wheel feedback guard exceeded";
                }
                if (state.enabled && !trackers_[i].valid())
                {
                    error = "Position continuity lost; stop and restart";
                }
                if (state.enabled && config_.mode == "bench" &&
                    std::abs(state.position[i] - bench_origin_[i]) > config_.bench_travel)
                {
                    error = "Bench travel limit reached";
                }
            }
            if (!error.empty())
            {
                fault(error);
                continue;
            }
            if (state.enabled)
            {
                if (active_relative_ && !step_relative(now))
                {
                    tick = SteadyClock::now();
                    continue;
                }
                std::array<double, 2> target, raw;
                bool expired = false;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    const bool source_fresh =
                        source_stamp_ != Deadline{} &&
                        now - source_stamp_ <= std::chrono::milliseconds(config_.command_timeout_ms);
                    const bool controller_fresh =
                        command_stamp_ != Deadline{} &&
                        now - command_stamp_ <= std::chrono::milliseconds(config_.mode == "base"
                                                                              ? config_.lateness_ms
                                                                              : config_.command_timeout_ms);
                    target = (state_.permitted && source_fresh && controller_fresh) ? target_
                                                                                    : std::array<double, 2>{};
                    expired = config_.mode == "base" && source_fresh && !controller_fresh &&
                              now - enabled_since_ > std::chrono::milliseconds(200);
                }
                if (expired)
                {
                    fault("ROS controller write deadline missed");
                    continue;
                }
                for (std::size_t i = 0; i < 2; ++i)
                {
                    const double step = config_.wheel_acceleration / config_.control_hz;
                    sent_[i] = std::clamp(target[i], sent_[i] - step, sent_[i] + step);
                    raw[i] = sent_[i] * config_.direction[i] * config_.reduction[i];
                }
                const auto result = bus_->send_velocity_pair(raw);
                if (result.code != ErrorCode::Ok)
                {
                    fault(result.message);
                    continue;
                }
                std::lock_guard<std::mutex> lock(mutex_);
                state_.target = sent_;
                state_.lifecycle = (std::abs(sent_[0]) + std::abs(sent_[1]) > 1e-6) ? 4 : 3;
            }
            else if (now >= next_idle_poll)
            {
                const auto result = bus_->poll_disabled_pair();
                next_idle_poll =
                    SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
                                             std::chrono::duration<double>(1 / config_.idle_poll_hz));
                if (result.code != ErrorCode::Ok)
                {
                    fault(result.message);
                    continue;
                }
            }
        }
        tick += std::chrono::duration_cast<SteadyClock::duration>(
            std::chrono::duration<double>(1 / config_.control_hz));
        if (!state.enabled && tick < SteadyClock::now())
        {
            tick = SteadyClock::now(); // 待机被调度延迟后不突发补发积压周期。
        }
        std::this_thread::sleep_until(tick);
    }
    bus_->stop_pair();
}
} // namespace robot_wheel_control
