#pragma once
#include <damiao_core/h55/types.hpp>
#include <cmath>
namespace robot_wheel_control
{
// 每个新sequence仅积分一次；无法唯一展开时失效，必须重建会话/重新标定。
class PositionTracker
{
  public:
    bool update(const damiao::MotorState &sample, double wrap, double maximum_speed, double resolution,
                double maximum_gap_s)
    {
        if (!sample.valid)
        {
            return false;
        }
        if (sample.sequence == sequence_)
        {
            return valid_;
        }
        if (!initialized_)
        {
            initialized_ = true;
            valid_ = true;
            raw_ = sample.output_position_rad;
            stamp_ = sample.received_at;
            sequence_ = sample.sequence;
            return true;
        }
        const double dt = std::chrono::duration<double>(sample.received_at - stamp_).count();
        double delta = sample.output_position_rad - raw_;
        if (dt <= 0 || dt > maximum_gap_s || (wrap > 0 && maximum_speed * dt + 2 * resolution >= wrap / 2))
        {
            valid_ = false;
        }
        if (wrap > 0)
        {
            delta = std::remainder(delta, wrap);
        }
        if (std::abs(delta) > maximum_speed * dt + 2 * resolution)
        {
            valid_ = false;
        }
        if (valid_)
        {
            position_ += delta;
        }
        sequence_ = sample.sequence;
        stamp_ = sample.received_at;
        raw_ = sample.output_position_rad;
        return valid_;
    }
    void reset()
    {
        *this = PositionTracker{};
    }
    double position() const
    {
        return position_;
    }
    bool valid() const
    {
        return valid_ && initialized_;
    }

  private:
    bool initialized_ = false, valid_ = false;
    std::uint64_t sequence_ = 0;
    damiao::Deadline stamp_{};
    double position_ = 0, raw_ = 0;
};
} // namespace robot_wheel_control
