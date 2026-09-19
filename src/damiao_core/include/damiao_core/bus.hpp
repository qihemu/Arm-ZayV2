#pragma once

#include <damiao_core/protocol.hpp>
#include <damiao_core/transport.hpp>

#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

namespace damiao
{

constexpr std::size_t max_motors = 6;
using MotorIndex = std::size_t;

// 地址与模式必须读回；型号、供电版本等身份信息由配置适配层补充。
struct MotorConfig
{
    std::string name;
    std::string model;
    std::string voltage_variant;
    MotorAddress address;
    ControlMode mode = ControlMode::PositionVelocity;
    MappingLimits mapping;
    // 被动监听可显式提供已核对的映射；主动连接仍需 synchronize_motor。
    bool mapping_confirmed = false;
    bool mode_confirmed = false;
    std::optional<std::uint32_t> firmware_version;
    // 输出轴的运动限制必须由使用方补齐；默认值禁止批量运动。
    std::optional<double> min_output_position_rad;
    std::optional<double> max_output_position_rad;
    std::optional<double> max_output_speed_rad_s;
};

struct BusConfig
{
    TransportConfig transport;
    // 不预设电机保护期限；调用者按总线测量与机械要求提供这些值。
    std::chrono::milliseconds feedback_timeout{0};
    std::chrono::milliseconds management_quiet_period{0};
};

enum class BusState
{
    Closed,
    Maintenance,
    Control,
    Fault
};

// 低频诊断快照；周期发送只更新计数与错误码，不拼接字符串。
struct BusDiagnostics
{
    std::uint64_t received_frames = 0;
    std::uint64_t rejected_frames = 0;
    std::uint64_t ambiguous_frames = 0;
    std::uint64_t transaction_timeouts = 0;
    std::uint64_t send_failures = 0;
    ErrorCode last_error = ErrorCode::Ok;
    std::string transport_error_detail;
};

// 固定最多六轴；注册/连接/管理串行化，快照允许与接收线程并发。
// 核心只启接收线程，不定时重发命令，不承诺物理停车或首次使能无跳变。
// 调试会话或 ROS 插件负责标定、变化率、目标有效期、使能时序与停止策略。
class DamiaoBus
{
public:
    // 接管传输后端所有权并初始化一个关闭状态的固定容量总线。
    explicit DamiaoBus(std::unique_ptr<ICanTransport> transport = std::make_unique<SocketCanTransport>());
    // 停止接收线程并释放主机资源，不向电机隐式发送命令。
    ~DamiaoBus();
    // 总线及其接收线程具有唯一所有权，因此禁止复制。
    DamiaoBus(const DamiaoBus&) = delete;
    // 禁止复制赋值，避免共享传输资源、线程和反馈缓存。
    DamiaoBus& operator=(const DamiaoBus&) = delete;

    // 在打开总线前注册并校验一台电机，返回固定缓存中的索引。
    Result<MotorIndex> register_motor(const MotorConfig& config);
    // 打开传输后端并启动唯一接收线程，成功后进入维护状态。
    Status open(const BusConfig& config);
    // Control 状态拒绝 close；先撤销运行许可并按上层策略完成停车/支撑交接。
    Status close();
    // 返回受互斥保护的总线生命周期状态。
    BusState state() const;
    // 返回接收、拒绝、超时和发送错误的低频诊断快照。
    BusDiagnostics diagnostics() const;
    // 返回指定电机当前可信配置的副本。
    Result<MotorConfig> motor_config(MotorIndex index) const;
    // 返回指定电机的最新反馈，并检查反馈有效性和新鲜度。
    Result<MotorState> snapshot(MotorIndex index) const;
    // 将全部电机快照写入调用方缓冲区，适合无分配周期读取。
    ErrorCode snapshot_into(MotorState* output, std::size_t count) const;

    // 仅维护路径，所有 deadline 均为整个操作的绝对截止时刻，不自动重试。
    // 并发管理立即返回 WouldBlock；销毁对象前调用方必须结束全部外部调用。
    // 读取单个寄存器，并将请求与响应事务串行化。
    Result<RegisterValue> read_parameter(MotorIndex index, std::uint8_t rid, Deadline deadline);
    // 读回控制模式寄存器 0x0A 并转换为 ControlMode。
    Result<ControlMode> read_control_mode(MotorIndex index, Deadline deadline);
    // 读回 PMAX/VMAX/TMAX 并组装 MappingLimits。
    Result<MappingLimits> read_mapping_limits(MotorIndex index, Deadline deadline);
    // 读回 TIMEOUT 寄存器并按 50μs/计数换算为毫秒。
    Result<std::chrono::milliseconds> read_communication_timeout(MotorIndex index, Deadline deadline);
    // 读回映射范围、控制模式和固件版本，原子提交可信配置。
    Status synchronize_motor(MotorIndex index, Deadline deadline);
    // 主动查询并返回一帧指定电机的普通状态反馈。
    Result<MotorState> query_state(MotorIndex index, Deadline deadline);
    // 执行读取旧值、写入和读回校验，保留非原子操作证据。
    ParameterWriteReport write_parameter_verified(MotorIndex index, std::uint8_t rid,
        const RegisterValue& value, Deadline deadline);
    // 写入并验证控制模式；成功后更新管理命令使用的模式。
    Status switch_mode(MotorIndex index, ControlMode mode, Deadline deadline);
    // switch_mode 的语义别名，支持写入 MIT 等全部 ControlMode 值。
    Status set_control_mode(MotorIndex index, ControlMode mode, Deadline deadline);
    // 单次管理锁内顺序写入 PMAX/VMAX/TMAX，任一步失败即停止。
    MappingLimitsWriteReport write_mapping_limits(MotorIndex index, const MappingLimits& limits,
        Deadline deadline);
    // 将毫秒超时换算为 TIMEOUT 计数并执行读-写-读回校验。
    ParameterWriteReport write_communication_timeout(MotorIndex index,
        std::chrono::milliseconds timeout, Deadline deadline);
    // 显式使能电机；上层须先完成已验证的保持目标和使能时序。
    Status enable(MotorIndex index, Deadline deadline);
    // 显式失能电机，并通过状态查询确认结果。
    Status disable(MotorIndex index, Deadline deadline);
    // 发送清错命令，并通过状态查询确认故障已清除。
    Status clear_error(MotorIndex index, Deadline deadline);
    // 将电机当前位置保存为零点，并确认命令响应。
    Status save_zero(MotorIndex index, Deadline deadline);
    // save_zero 的语义别名。
    Status save_zero_position(MotorIndex index, Deadline deadline);
    // 在维护状态且电机失能时保存当前参数配置。
    Status save_parameters(MotorIndex index, Deadline deadline);
    // 清除事务超时等可恢复故障，回到维护态并作废全部反馈缓存。
    Status recover_maintenance();

    // 只开放发送许可，不使能、不发送目标；要求全部反馈新鲜、使能且配置可信。
    Status begin_control();
    // 只撤销发送许可，不证明已停车；不会隐式失能承重轴。
    Status end_control();
    // 与管理互斥；先全量检查再逐帧发送，results 必须预分配，未发送为 NotExecuted。
    // 有发送失败则锁存 Fault，不重试、不回滚、不保留队列。
    ErrorCode send_position_velocity_batch(const PositionVelocityCommand* commands,
        std::size_t count, ErrorCode* results);

private:
    struct Pending
    {
        MotorIndex index = 0;
        std::uint8_t operation = 0;
        std::uint8_t rid = 0;
        Deadline sent_at{};
        Deadline deadline{};
        bool active = false;
        bool done = false;
        ErrorCode code = ErrorCode::Ok;
        std::optional<RegisterValue> value;
    };

    // 检查管理操作所需的总线状态、电机索引及可选失能条件。
    Status management_gate(MotorIndex index, bool require_disabled) const;
    // 安装单个待处理请求，发送帧并等待接收线程匹配响应。
    Status transact(MotorIndex index, const CanFrame& request, std::uint8_t operation,
        std::uint8_t rid, Deadline deadline, std::optional<RegisterValue>& value,
        bool* sent = nullptr);
    // 在已持有管理互斥时读取寄存器。
    Result<RegisterValue> read_unlocked(MotorIndex index, std::uint8_t rid, Deadline deadline);
    // 在已持有管理互斥时查询电机状态。
    Result<MotorState> query_unlocked(MotorIndex index, Deadline deadline);
    // 在已持有管理互斥时执行可验证的寄存器写事务。
    ParameterWriteReport write_unlocked(MotorIndex index, std::uint8_t rid,
        const RegisterValue& value, Deadline deadline, bool verify_disabled = true);
    // 在已持有管理互斥时发送管理命令并核对执行结果。
    Status command_unlocked(MotorIndex index, ManagementCommand command, Deadline deadline);
    // 持续接收传输帧，并将帧分派给事务等待者或反馈缓存。
    void receive_loop();
    // 分类并校验单帧，更新匹配事务、反馈缓存和诊断计数。
    void route_frame(const CanFrame& frame);
    // 停止接收线程并关闭传输后端，供关闭和失败回滚共用。
    Status shutdown_resources();
    // 使指定电机的缓存反馈失效，防止配置变化后复用旧语义。
    void invalidate(MotorIndex index);
    // 检查指定电机反馈是否存在、配置匹配且未超过超时期限。
    ErrorCode feedback_code(MotorIndex index, Deadline now) const;

    std::unique_ptr<ICanTransport> transport_;
    mutable std::mutex operation_mutex_;
    mutable std::mutex cache_mutex_;
    std::condition_variable received_;
    std::array<MotorConfig, max_motors> motors_{};
    std::array<MotorState, max_motors> states_{};
    std::array<std::uint64_t, max_motors> revisions_{};
    std::array<Deadline, max_motors> valid_after_{};
    std::array<int, 16> esc_index_{};
    std::array<int, 2048> mst_index_{};
    std::size_t motor_count_ = 0;
    BusConfig config_;
    BusState state_ = BusState::Closed;
    Pending pending_;
    BusDiagnostics diagnostics_;
    Deadline last_receive_{};
    std::atomic<bool> stop_{true};
    std::thread receiver_;
};

}  // namespace damiao
