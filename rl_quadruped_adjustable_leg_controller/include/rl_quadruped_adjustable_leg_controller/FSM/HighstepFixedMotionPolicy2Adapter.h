#ifndef HIGHSTEP_FIXED_MOTION_POLICY2_ADAPTER_H
#define HIGHSTEP_FIXED_MOTION_POLICY2_ADAPTER_H

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace highstep_fixed_motion
{

class Policy2Adapter
{
public:
    static constexpr double kFullPushVx = 0.7200000286;
    static constexpr double kFullPushSteps = 59.0;
    static constexpr int kSequenceSteps = 138;
    static constexpr double kTerminalEpsilon = 1.0e-6;
    static constexpr std::array<double, 7> kPhaseEdges = {0.0, 0.15, 0.35, 0.55, 0.70, 0.85, 1.0};

    struct CommandSample
    {
        double raw_joystick_vx = 0.0;
        double model_vx = 0.0;
        int diagnostic_step = -1;
        bool diagnostic_armed = false;
        bool sequence_complete = false;
    };

    void reset() noexcept
    {
        phase_ = 0.0;
        sequence_step_ = -1;
        sequence_state_ = SequenceState::Idle;
    }

    bool trigger() noexcept
    {
        if (sequence_state_ != SequenceState::Idle)
        {
            return false;
        }
        sequence_step_ = 0;
        sequence_state_ = SequenceState::Running;
        return true;
    }

    CommandSample resolveCommand(const double raw_joystick_vx)
    {
        CommandSample sample;
        sample.raw_joystick_vx = raw_joystick_vx;
        sample.diagnostic_armed = sequence_state_ == SequenceState::Running;
        sample.sequence_complete = sequence_state_ == SequenceState::Complete;
        if (sequence_state_ == SequenceState::Running)
        {
            sample.diagnostic_step = sequence_step_;
            if (sequence_step_ >= 21 && sequence_step_ < 80)
            {
                sample.model_vx = kFullPushVx;
            }
            advance(sample.model_vx);
            ++sequence_step_;
            if (sequence_step_ >= kSequenceSteps)
            {
                sequence_state_ = SequenceState::Complete;
                sample.sequence_complete = true;
            }
        }
        return sample;
    }

    double advance(const double vx)
    {
        if (!std::isfinite(vx))
        {
            throw std::invalid_argument("fixed-motion phase requires a finite joystick vx");
        }
        phase_ = std::clamp(phase_ + std::max(vx, 0.0) / kFullPushVx / kFullPushSteps, 0.0, 1.0);
        if (phase_ >= 1.0 - kTerminalEpsilon)
        {
            phase_ = 1.0;
        }
        return phase_;
    }

    [[nodiscard]] double phase() const noexcept { return phase_; }
    [[nodiscard]] bool running() const noexcept { return sequence_state_ == SequenceState::Running; }
    [[nodiscard]] bool complete() const noexcept { return sequence_state_ == SequenceState::Complete; }
    [[nodiscard]] int sequenceStep() const noexcept { return sequence_step_; }

    [[nodiscard]] std::array<float, 8> features() const
    {
        std::array<float, 8> result{};
        result[0] = static_cast<float>(phase_);
        for (std::size_t index = 0; index < 6; ++index)
        {
            const double lower = kPhaseEdges[index];
            const double upper = kPhaseEdges[index + 1];
            const bool selected = phase_ >= lower && (index < 5 ? phase_ < upper : phase_ <= upper);
            if (selected)
            {
                result[1 + index] = 1.0F;
                result[7] = static_cast<float>(std::clamp((phase_ - lower) / (upper - lower), 0.0, 1.0));
                break;
            }
        }
        return result;
    }

    [[nodiscard]] std::array<float, 4> boxBiasMeters() const noexcept
    {
        if (phase_ <= 0.0 || phase_ >= 1.0 - kTerminalEpsilon)
        {
            return {0.0F, 0.0F, 0.0F, 0.0F};
        }
        if (phase_ < 0.15)
        {
            return {-0.020F, -0.020F, 0.002F, 0.002F};
        }
        return {0.002F, 0.002F, -0.022F, -0.022F};
    }

private:
    enum class SequenceState { Idle, Running, Complete };
    double phase_ = 0.0;
    int sequence_step_ = -1;
    SequenceState sequence_state_ = SequenceState::Idle;
};

}  // namespace highstep_fixed_motion

#endif  // HIGHSTEP_FIXED_MOTION_POLICY2_ADAPTER_H
