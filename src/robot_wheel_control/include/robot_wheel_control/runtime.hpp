#pragma once
#include <robot_wheel_control/configuration.hpp>
#include <robot_wheel_control/position_tracker.hpp>
#include <robot_wheel_control/wheel_backend.hpp>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <thread>

namespace robot_wheel_control
{
enum class Operation
{
    Enable,
    Disable,
    Stop,
    Clear,
    Relative
};
struct OperationResult
{
    std::string id, reason;
    int status = 1; // PENDING=1, SUCCEEDED=2, FAILED=3, CANCELLED=4
};
struct RuntimeState
{
    std::array<damiao::MotorState, 2> motors;
    std::array<double, 2> position{}, velocity{}, target{};
    bool command_fresh = false;
    bool configured = false, enabled = false, permitted = false, fault = false, standstill = false,
         position_valid = false;
    int lifecycle = 0;
    std::uint64_t sequence = 0, fault_sequence = 0;
    double path = 0;
    std::string reason = "Starting", session, last_request;
    int last_result = 0;
    bool relative_active = false;
    std::string relative_id;
    RelativeGoal relative_goal;
    std::array<double, 2> relative_target{}, relative_travel{};
    double relative_measured = 0;
};
// 每个插件/台架进程持有一个实例；三个工作线程分别负责RX、TX及管理调度。
class WheelRuntime
{
  public:
    explicit WheelRuntime(Configuration config, std::unique_ptr<damiao::ICanTransport> transport = nullptr);
    ~WheelRuntime();
    void start();
    void shutdown();
    RuntimeState snapshot() const;
    const Configuration &configuration() const
    {
        return config_;
    }
    bool command(const std::array<double, 2> &speed, bool controller_write = false);
    void authorize_source(); // 仅新鲜、经仲裁的TwistStamped调用；不使能。
    bool submit(Operation operation, const std::string &session, const std::string &id,
                std::uint64_t expected_fault, bool disable_after, std::string &reason,
                const RelativeGoal &goal = {});
    bool relative_heartbeat(const std::string &session, const std::string &id);
    OperationResult result(const std::string &id) const;
    std::vector<OperationResult> take_events();
    void request_fault(const std::string &reason);

  private:
    struct Request
    {
        Operation operation;
        std::string id;
        bool disable_after;
        std::uint64_t generation;
        RelativeGoal goal;
    };
    void receive_loop();
    void transmit_loop();
    void management_loop();
    damiao::Status perform(const Request &request);
    damiao::Status controlled_stop(bool disable);
    void finish(const Request &request, const damiao::Status &status);
    void prune_history_locked();
    void update_measurements();
    void fault(const std::string &reason);
    damiao::Status start_relative(const Request &request);
    bool step_relative(damiao::Deadline now);
    void end_relative(damiao::Status outcome, bool already_stopped = false);
    bool relative_cancelled(const Request &request) const;
    Configuration config_;
    std::unique_ptr<IWheelBackend> bus_;
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    RuntimeState state_;
    std::string fault_reason_; // 保留首次故障，不被后续停车成功文案覆盖。
    std::array<PositionTracker, 2> trackers_;
    std::array<double, 2> target_{}, sent_{}, bench_origin_{};
    damiao::Deadline command_stamp_{}, source_stamp_{}, enabled_since_{};
    std::array<damiao::Deadline, 2> stationary_since_{};
    std::array<double, 2> stationary_anchor_{};
    std::deque<Request> requests_;
    std::optional<Request> stop_request_;
    std::map<std::string, OperationResult> results_;
    std::map<std::string, std::string> fingerprints_;
    std::deque<std::string> completed_;
    std::vector<OperationResult> events_;
    std::function<void()> tx_job_;
    std::thread receiver_, transmitter_, manager_;
    std::atomic<bool> running_{false}, receiving_{false}, stop_pending_{false};
    std::uint64_t stop_generation_ = 0;
    // 相对运动算法只由TX线程访问；共享心跳和状态仍受mutex_保护。
    std::optional<Request> active_relative_;
    std::array<double, 2> relative_origin_{}, relative_caps_{}, progress_anchor_{};
    std::array<damiao::Deadline, 2> progress_since_{};
    damiao::Deadline relative_started_{}, relative_settled_{}, heartbeat_stamp_{};
};
} // namespace robot_wheel_control
