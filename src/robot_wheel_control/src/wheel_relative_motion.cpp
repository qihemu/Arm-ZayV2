#include <robot_wheel_control/runtime.hpp>
#include <sstream>

namespace robot_wheel_control
{
using namespace damiao;

bool WheelRuntime::relative_heartbeat(const std::string &session, const std::string &id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.relative_active || state_.session != session || state_.relative_id != id || state_.fault)
    {
        return false;
    }
    heartbeat_stamp_ = SteadyClock::now();
    return true;
}

bool WheelRuntime::relative_cancelled(const Request &r) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return !running_ || state_.fault || stop_generation_ != r.generation ||
           SteadyClock::now() - heartbeat_stamp_ > std::chrono::milliseconds(config_.relative.heartbeat_ms);
}

Status WheelRuntime::start_relative(const Request &r)
{
    if (relative_cancelled(r))
    {
        return {ErrorCode::NotExecuted, "Relative start cancelled or heartbeat expired"};
    }
    std::array<double, 2> target, caps;
    for (std::size_t i = 0; i < 2; ++i)
    {
        double scale = 1;
        if (r.goal.kind == 1)
        {
            scale = 1 / config_.radius[i];
        }
        if (r.goal.kind == 2)
        {
            scale = (i == 0 ? -1 : 1) * config_.separation / (2 * config_.radius[i]);
        }
        target[i] = r.goal.value * scale;
        caps[i] = r.goal.max_speed * std::abs(scale);
        // 台架允许按初始几何换算小目标，但绝不扩大原有悬空轮角行程范围。
        if (config_.mode == "bench" && std::abs(target[i]) > config_.bench_travel * 0.8)
        {
            return {ErrorCode::InvalidCommand, "Suspended bench target exceeds 80% of wheel travel limit; "
                                               "ground motion requires commissioned relative mode"};
        }
        if (!std::isfinite(target[i]) || !std::isfinite(caps[i]) || caps[i] <= 0 ||
            std::abs(target[i]) <= 2 * config_.relative.wheel_tolerance)
        {
            return {ErrorCode::InvalidCommand, "Target below wheel resolution/tolerance or geometry invalid"};
        }
    }
    // 双轮共同缩放，保持差速运动的比例，而非分别截断改变转弯半径。
    const double ratio = std::min(1.0, config_.wheel_speed / std::max(caps[0], caps[1]));
    for (auto &cap : caps)
    {
        cap *= ratio;
    }
    const double minimum_time = std::max(std::abs(target[0]) / caps[0], std::abs(target[1]) / caps[1]);
    if (minimum_time + 2 * config_.relative.settle_ms / 1000.0 >= r.goal.timeout_s)
    {
        return {ErrorCode::InvalidCommand, "Timeout too short for target and configured speed limit"};
    }
    // 重用新鲜失能确认、显式使能和停止代次检查，不能绕过原有驱动保护。
    auto enable = r;
    enable.operation = Operation::Enable;
    const auto enabled = perform(enable);
    if (enabled.code != ErrorCode::Ok)
    {
        return enabled;
    }
    update_measurements();
    if (relative_cancelled(r) || !trackers_[0].valid() || !trackers_[1].valid())
    {
        controlled_stop(true);
        return {ErrorCode::NotExecuted, "Relative start cancelled or position unavailable"};
    }
    active_relative_ = r;
    relative_origin_ = snapshot().position;
    relative_caps_ = caps;
    progress_anchor_ = {};
    relative_started_ = SteadyClock::now();
    relative_settled_ = {};
    progress_since_.fill(relative_started_);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.relative_target = target;
        state_.relative_travel = {};
        state_.relative_measured = 0;
        state_.reason = "Relative motion running";
    }
    return {};
}

bool WheelRuntime::step_relative(Deadline now)
{
    const auto request = *active_relative_;
    if (relative_cancelled(request))
    {
        end_relative({ErrorCode::NotExecuted, "Relative motion cancelled: stop, fault or heartbeat expired"});
        return false;
    }
    if (now - relative_started_ > std::chrono::duration<double>(request.goal.timeout_s))
    {
        end_relative({ErrorCode::Timeout, "Relative motion deadline exceeded"});
        return false;
    }
    const auto s = snapshot();
    std::array<double, 2> travel;
    bool settled = true;
    for (std::size_t i = 0; i < 2; ++i)
    {
        travel[i] = s.position[i] - relative_origin_[i];
        const double error = s.relative_target[i] - travel[i];
        const bool near = std::abs(error) <= config_.relative.wheel_tolerance * 0.5;
        settled = settled && near && std::abs(s.velocity[i]) <= config_.relative.settled_speed &&
                  std::abs(sent_[i]) < 1e-6;
        if (near || std::abs(travel[i] - progress_anchor_[i]) >= config_.relative.progress_step)
        {
            progress_anchor_[i] = travel[i];
            progress_since_[i] = now;
        }
        if (now - progress_since_[i] > std::chrono::duration<double>(config_.relative.stall_timeout_s))
        {
            end_relative({ErrorCode::Timeout, "Relative motion stalled: wheel=" + std::to_string(i)});
            return false;
        }
    }
    const auto velocity = relative_velocity(s.relative_target, travel, relative_caps_,
                                            config_.wheel_acceleration, config_.relative);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.relative_travel = travel;
        if (request.goal.kind == 1)
        {
            state_.relative_measured = (travel[0] * config_.radius[0] + travel[1] * config_.radius[1]) / 2;
        }
        else if (request.goal.kind == 2)
        {
            state_.relative_measured =
                (travel[1] * config_.radius[1] - travel[0] * config_.radius[0]) / config_.separation;
        }
        else
        {
            state_.relative_measured = (travel[0] + travel[1]) / 2;
        }
        // TX拥有目标，心跳拥有任务许可；没有新心跳时不靠内部刷新续命。
        target_ = velocity;
        source_stamp_ = now;
        command_stamp_ = now;
    }
    if (!settled)
    {
        relative_settled_ = {};
    }
    else if (relative_settled_ == Deadline{})
    {
        relative_settled_ = now;
    }
    else if (now - relative_settled_ >= std::chrono::milliseconds(config_.relative.settle_ms))
    {
        end_relative({});
        return false;
    }
    return true;
}

void WheelRuntime::end_relative(Status outcome, bool already_stopped)
{
    const auto request = *active_relative_;
    active_relative_.reset(); // controlled_stop不重复生成取消结果。
    if (!already_stopped)
    {
        const auto stopped = controlled_stop(true);
        if (stopped.code != ErrorCode::Ok)
        {
            outcome = stopped;
        }
    }
    // 到位必须在FD之后再次观察；不能把失能前位置或单个零速帧当最终到位。
    const auto end =
        SteadyClock::now() +
        std::chrono::milliseconds(std::max(config_.stop_timeout_ms, config_.relative.settle_ms + 500));
    Deadline stable_since{};
    bool final_confirmed = false;
    while (outcome.code == ErrorCode::Ok && SteadyClock::now() < end)
    {
        if (relative_cancelled(request))
        {
            outcome = {ErrorCode::NotExecuted, "Relative motion cancelled during final observation"};
            break;
        }
        const auto refreshed = bus_->poll_disabled_pair();
        if (refreshed.code != ErrorCode::Ok)
        {
            fault(refreshed.message);
            outcome = refreshed;
            break;
        }
        update_measurements();
        const auto s = snapshot();
        bool settled = true;
        for (std::size_t i = 0; i < 2; ++i)
        {
            const auto age = SteadyClock::now() - s.motors[i].received_at;
            if (!trackers_[i].valid() || !s.motors[i].valid || s.motors[i].raw_status != 0 ||
                age > config_.bus.feedback_timeout)
            {
                std::ostringstream detail;
                detail << "Final feedback invalid: wheel=" << i << " tracker=" << trackers_[i].valid()
                       << " status=" << int(s.motors[i].raw_status)
                       << " age_ms=" << std::chrono::duration<double, std::milli>(age).count();
                outcome = {ErrorCode::StaleFeedback, detail.str()};
                break;
            }
            settled = settled &&
                      std::abs(s.relative_target[i] - (s.position[i] - relative_origin_[i])) <=
                          config_.relative.wheel_tolerance &&
                      std::abs(s.velocity[i]) <= config_.relative.settled_speed;
        }
        // 允许失能瞬间滤波速度收敛，但必须随后连续满足同一误差/速度条件。
        if (!settled)
        {
            stable_since = {};
        }
        else if (stable_since == Deadline{})
        {
            stable_since = SteadyClock::now();
        }
        else if (SteadyClock::now() - stable_since >= std::chrono::milliseconds(config_.relative.settle_ms))
        {
            final_confirmed = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(1 / config_.control_hz));
    }
    if (outcome.code == ErrorCode::Ok && !final_confirmed)
    {
        const auto s = snapshot();
        std::ostringstream detail;
        detail << "Final disabled settling timeout: errors="
               << s.relative_target[0] - (s.position[0] - relative_origin_[0]) << ','
               << s.relative_target[1] - (s.position[1] - relative_origin_[1])
               << " velocities=" << s.velocity[0] << ',' << s.velocity[1];
        outcome = {ErrorCode::Timeout, detail.str()};
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (std::size_t i = 0; i < 2; ++i)
        {
            state_.relative_travel[i] = state_.position[i] - relative_origin_[i];
        }
        const auto &d = state_.relative_travel;
        state_.relative_measured =
            request.goal.kind == 1
                ? (d[0] * config_.radius[0] + d[1] * config_.radius[1]) / 2
                : (request.goal.kind == 2
                       ? (d[1] * config_.radius[1] - d[0] * config_.radius[0]) / config_.separation
                       : (d[0] + d[1]) / 2);
        if (outcome.code == ErrorCode::Ok)
        {
            std::ostringstream result;
            result << "Relative target reached; drivers disabled; measured=" << state_.relative_measured
                   << " error=" << request.goal.value - state_.relative_measured;
            if (config_.mode == "bench" && request.goal.kind != 3)
            {
                result << "; suspended encoder-equivalent only, not measured chassis travel";
            }
            outcome.message = result.str();
        }
        if (!state_.fault)
        {
            state_.reason = outcome.message;
        }
    }
    finish(request, outcome);
}
} // namespace robot_wheel_control
