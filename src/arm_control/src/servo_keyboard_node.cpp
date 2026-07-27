/**
 * @file servo_keyboard_node.cpp
 * @brief 键盘遥操作节点：向 MoveIt Servo 发布 TwistStamped 速度指令
 *
 * 采用按键活跃超时模式，支持组合按键（如 ↑+→ 对角线运动）。
 * ros2 launch 下 stdin 通常不是 TTY，会自动回退读取 /dev/tty。
 */

#include <atomic>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <signal.h>
#include <stdio.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace arm_control
{

enum class MotionKey : uint8_t
{
    LIN_X_POS,
    LIN_X_NEG,
    LIN_Y_POS,
    LIN_Y_NEG,
    LIN_Z_POS,
    LIN_Z_NEG,
    ANG_X_POS,
    ANG_X_NEG,
    ANG_Y_POS,
    ANG_Y_NEG,
    ANG_Z_POS,
    ANG_Z_NEG,
};

class KeyboardReader
{
public:
    KeyboardReader()
    {
        file_descriptor_ = STDIN_FILENO;
        owns_file_descriptor_ = false;

        if (!isatty(file_descriptor_))
        {
            // ros2 launch 子进程 stdin 非 TTY；从当前终端设备读取按键
            file_descriptor_ = ::open("/dev/tty", O_RDWR);
            owns_file_descriptor_ = true;
            if (file_descriptor_ < 0 || !isatty(file_descriptor_))
            {
                if (file_descriptor_ >= 0)
                {
                    ::close(file_descriptor_);
                }
                throw std::runtime_error(
                    "No interactive TTY available. Run from a terminal, or use: "
                    "ros2 run arm_control servo_keyboard_node");
            }
        }

        tcgetattr(file_descriptor_, &cooked_);
        struct termios raw;
        std::memcpy(&raw, &cooked_, sizeof(struct termios));
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(file_descriptor_, TCSANOW, &raw);
    }

    ~KeyboardReader()
    {
        shutdown();
    }

    void shutdown()
    {
        if (!restored_)
        {
            tcsetattr(file_descriptor_, TCSANOW, &cooked_);
            restored_ = true;
        }
        if (owns_file_descriptor_ && file_descriptor_ >= 0)
        {
            ::close(file_descriptor_);
            file_descriptor_ = -1;
            owns_file_descriptor_ = false;
        }
    }

    bool readByte(char* byte, int timeout_ms)
    {
        fd_set set;
        FD_ZERO(&set);
        FD_SET(file_descriptor_, &set);

        struct timeval timeout;
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;

        const int result = select(file_descriptor_ + 1, &set, nullptr, nullptr, &timeout);
        if (result <= 0)
        {
            return false;
        }

        const ssize_t nbytes = ::read(file_descriptor_, byte, 1);
        return nbytes == 1;
    }

private:
    int file_descriptor_{ -1 };
    bool owns_file_descriptor_{ false };
    struct termios cooked_{};
    bool restored_{ false };
};

class KeyState
{
public:
    void mark(MotionKey key, const rclcpp::Time& now)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_seen_[key] = now;
    }

    void clearAll()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_seen_.clear();
    }

    bool isActive(MotionKey key, const rclcpp::Time& now, double timeout_sec) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = last_seen_.find(key);
        if (it == last_seen_.end())
        {
            return false;
        }
        return (now - it->second).seconds() < timeout_sec;
    }

private:
    mutable std::mutex mutex_;
    std::map<MotionKey, rclcpp::Time> last_seen_;
};

KeyboardReader* g_keyboard_reader = nullptr;

class ServoKeyboardNode : public rclcpp::Node
{
public:
    ServoKeyboardNode() : Node("servo_keyboard_node")
    {
        twist_command_topic_ = declare_parameter<std::string>("twist_command_topic", "/servo_node/delta_twist_cmds");
        command_frame_id_ = declare_parameter<std::string>("command_frame_id", "base_link");
        linear_speed_ = declare_parameter<double>("linear_speed", 0.05);
        angular_speed_ = declare_parameter<double>("angular_speed", 0.3);
        publish_rate_ = declare_parameter<double>("publish_rate", 20.0);
        key_timeout_ = declare_parameter<double>("key_timeout", 0.15);

        twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(twist_command_topic_, 10);

        printHelp();

        keyboard_reader_ = std::make_unique<KeyboardReader>();
        g_keyboard_reader = keyboard_reader_.get();
        running_ = true;
        keyboard_thread_ = std::thread([this]() { keyboardLoop(); });

        const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
        publish_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { publishTwist(); });
    }

    ~ServoKeyboardNode() override
    {
        running_ = false;
        if (keyboard_thread_.joinable())
        {
            keyboard_thread_.join();
        }
        if (keyboard_reader_)
        {
            keyboard_reader_->shutdown();
            g_keyboard_reader = nullptr;
        }
    }

    void requestStop()
    {
        running_ = false;
        rclcpp::shutdown();
    }

private:
    static void printHelp()
    {
        puts("MoveIt Servo keyboard teleop");
        puts("----------------------------");
        puts("Translation (planning frame):");
        puts("  Up/Down     : +X / -X");
        puts("  Right/Left  : +Y / -Y");
        puts("  ; / .       : +Z / -Z");
        puts("Rotation:");
        puts("  I / K       : +Roll / -Roll");
        puts("  J / L       : +Pitch / -Pitch");
        puts("  U / O       : +Yaw / -Yaw");
        puts("Other:");
        puts("  Space       : emergency stop");
        puts("  Q           : quit");
        puts("Hold multiple keys for combined motion.");
    }

    void keyboardLoop()
    {
        std::vector<char> escape_buffer;

        while (running_ && rclcpp::ok())
        {
            char byte = 0;
            if (!keyboard_reader_->readByte(&byte, 50))
            {
                continue;
            }

            if (byte == '\x1b')
            {
                escape_buffer.clear();
                escape_buffer.push_back(byte);
                continue;
            }

            if (!escape_buffer.empty())
            {
                escape_buffer.push_back(byte);
                if (escape_buffer.size() < 3)
                {
                    continue;
                }

                if (escape_buffer[0] == '\x1b' && escape_buffer[1] == '[')
                {
                    switch (escape_buffer[2])
                    {
                        case 'A':
                            markKey(MotionKey::LIN_X_POS);
                            break;
                        case 'B':
                            markKey(MotionKey::LIN_X_NEG);
                            break;
                        case 'C':
                            markKey(MotionKey::LIN_Y_POS);
                            break;
                        case 'D':
                            markKey(MotionKey::LIN_Y_NEG);
                            break;
                        default:
                            break;
                    }
                }
                escape_buffer.clear();
                continue;
            }

            switch (byte)
            {
                case ';':
                    markKey(MotionKey::LIN_Z_POS);
                    break;
                case '.':
                    markKey(MotionKey::LIN_Z_NEG);
                    break;
                case 'i':
                case 'I':
                    markKey(MotionKey::ANG_X_POS);
                    break;
                case 'k':
                case 'K':
                    markKey(MotionKey::ANG_X_NEG);
                    break;
                case 'j':
                case 'J':
                    markKey(MotionKey::ANG_Y_POS);
                    break;
                case 'l':
                case 'L':
                    markKey(MotionKey::ANG_Y_NEG);
                    break;
                case 'u':
                case 'U':
                    markKey(MotionKey::ANG_Z_POS);
                    break;
                case 'o':
                case 'O':
                    markKey(MotionKey::ANG_Z_NEG);
                    break;
                case ' ':
                    key_state_.clearAll();
                    RCLCPP_INFO(get_logger(), "Emergency stop: all velocities cleared");
                    break;
                case 'q':
                case 'Q':
                    RCLCPP_INFO(get_logger(), "Quit requested");
                    requestStop();
                    break;
                default:
                    break;
            }
        }
    }

    void markKey(MotionKey key)
    {
        key_state_.mark(key, now());
    }

    void publishTwist()
    {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = command_frame_id_;

        const rclcpp::Time current_time = now();

        if (key_state_.isActive(MotionKey::LIN_X_POS, current_time, key_timeout_))
        {
            msg.twist.linear.x += linear_speed_;
        }
        if (key_state_.isActive(MotionKey::LIN_X_NEG, current_time, key_timeout_))
        {
            msg.twist.linear.x -= linear_speed_;
        }
        if (key_state_.isActive(MotionKey::LIN_Y_POS, current_time, key_timeout_))
        {
            msg.twist.linear.y += linear_speed_;
        }
        if (key_state_.isActive(MotionKey::LIN_Y_NEG, current_time, key_timeout_))
        {
            msg.twist.linear.y -= linear_speed_;
        }
        if (key_state_.isActive(MotionKey::LIN_Z_POS, current_time, key_timeout_))
        {
            msg.twist.linear.z += linear_speed_;
        }
        if (key_state_.isActive(MotionKey::LIN_Z_NEG, current_time, key_timeout_))
        {
            msg.twist.linear.z -= linear_speed_;
        }
        if (key_state_.isActive(MotionKey::ANG_X_POS, current_time, key_timeout_))
        {
            msg.twist.angular.x += angular_speed_;
        }
        if (key_state_.isActive(MotionKey::ANG_X_NEG, current_time, key_timeout_))
        {
            msg.twist.angular.x -= angular_speed_;
        }
        if (key_state_.isActive(MotionKey::ANG_Y_POS, current_time, key_timeout_))
        {
            msg.twist.angular.y += angular_speed_;
        }
        if (key_state_.isActive(MotionKey::ANG_Y_NEG, current_time, key_timeout_))
        {
            msg.twist.angular.y -= angular_speed_;
        }
        if (key_state_.isActive(MotionKey::ANG_Z_POS, current_time, key_timeout_))
        {
            msg.twist.angular.z += angular_speed_;
        }
        if (key_state_.isActive(MotionKey::ANG_Z_NEG, current_time, key_timeout_))
        {
            msg.twist.angular.z -= angular_speed_;
        }

        twist_pub_->publish(msg);
    }

    std::string twist_command_topic_;
    std::string command_frame_id_;
    double linear_speed_{ 0.05 };
    double angular_speed_{ 0.3 };
    double publish_rate_{ 20.0 };
    double key_timeout_{ 0.15 };

    KeyState key_state_;
    std::unique_ptr<KeyboardReader> keyboard_reader_;
    std::atomic<bool> running_{ false };
    std::thread keyboard_thread_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

void signalHandler(int /*sig*/)
{
    if (g_keyboard_reader != nullptr)
    {
        g_keyboard_reader->shutdown();
    }
    rclcpp::shutdown();
}

}  // namespace arm_control

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<arm_control::ServoKeyboardNode>();
    signal(SIGINT, arm_control::signalHandler);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
