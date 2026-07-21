#ifndef HIGHSTEP_TEACHER_VIRTUAL_TERRAIN_ADAPTER_H
#define HIGHSTEP_TEACHER_VIRTUAL_TERRAIN_ADAPTER_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace highstep_virtual_terrain
{

struct State
{
    double longitudinal_progress = 0.0;
    double lateral_offset = 0.0;
    double relative_yaw = 0.0;
    double body_height_offset = 0.0;
    double confidence = 1.0;
};

class Adapter
{
public:
    void reset(const std::array<double, 4>& quaternion_wxyz, const std::array<double, 16>& joint_position)
    {
        state_ = {};
        state_.confidence = 1.0;
        initial_yaw_ = yaw(quaternion_wxyz);
        initial_height_proxy_ = heightProxy(joint_position);
        inertial_velocity_x_ = 0.0;
        previous_phase_ = 0.0;
        low_confidence_steps_ = 0;
        base_domain_valid_ = true;
        height_in_phase_envelope_ = true;
        yaw_in_phase_envelope_ = true;
        expected_height_ = 0.0;
        height_residual_ = 0.0;
        expected_yaw_ = 0.0;
        yaw_residual_ = 0.0;
    }

    void update(
        const double command_x,
        const double command_y,
        const double command_yaw,
        const std::array<double, 4>& quaternion_wxyz,
        const std::array<double, 3>& body_acceleration,
        const std::array<double, 16>& joint_position,
        const std::array<double, 16>& joint_velocity,
        const double measured_body_velocity_x,
        const bool measured_velocity_available,
        const double dt)
    {
        if (!(std::isfinite(dt) && dt > 0.0 && dt <= 0.1))
        {
            base_domain_valid_ = false;
            state_.confidence = 0.0;
            ++low_confidence_steps_;
            return;
        }

        // Command is only the dead-reckoning prediction.  Existing onboard
        // velocity (when available), IMU acceleration and joint/kinematic
        // consistency correct the prediction; none is a simulator world pose.
        const double clipped_ax = std::clamp(body_acceleration[0], -3.0, 3.0);
        inertial_velocity_x_ = 0.96 * inertial_velocity_x_ + clipped_ax * dt;
        double corrected_vx = command_x;
        if (measured_velocity_available && std::isfinite(measured_body_velocity_x))
        {
            corrected_vx = 0.55 * command_x + 0.45 * measured_body_velocity_x;
        }
        else
        {
            corrected_vx = 0.80 * command_x + 0.20 * inertial_velocity_x_;
        }
        state_.longitudinal_progress += corrected_vx * dt;
        state_.lateral_offset += command_y * dt;
        state_.relative_yaw = wrapAngle(yaw(quaternion_wxyz) - initial_yaw_);
        // Preserve real joystick yaw as a prediction while IMU yaw supplies
        // the correction. This also updates while the stick is released.
        state_.relative_yaw = wrapAngle(0.85 * state_.relative_yaw + 0.15 * (state_.relative_yaw + command_yaw * dt));
        state_.body_height_offset = heightProxy(joint_position) - initial_height_proxy_;

        bool finite = true;
        double max_joint_speed = 0.0;
        for (std::size_t i = 0; i < joint_position.size(); ++i)
        {
            finite = finite && std::isfinite(joint_position[i]) && std::isfinite(joint_velocity[i]);
            max_joint_speed = std::max(max_joint_speed, std::abs(joint_velocity[i]));
        }
        // Height is intentionally excluded here. Normal climbing changes the
        // kinematic height proxy by more than 5 cm. It is checked later against
        // the immutable envelope for the causal canonical phase.
        base_domain_valid_ = finite
            && std::abs(state_.lateral_offset) <= 0.04
            && max_joint_speed < 35.0;
    }

    bool acceptReferenceEnvelope(
        const double phase,
        const double expected_height,
        const double height_lower,
        const double height_upper,
        const double expected_yaw,
        const double yaw_lower,
        const double yaw_upper)
    {
        const bool phase_valid = std::isfinite(phase) && phase >= -1.0e-6 && phase <= 1.0 + 1.0e-6
            && std::abs(phase - previous_phase_) <= 0.16;
        const bool envelope_finite = std::isfinite(expected_height) && std::isfinite(height_lower)
            && std::isfinite(height_upper) && height_lower <= expected_height && expected_height <= height_upper
            && std::isfinite(expected_yaw) && std::isfinite(yaw_lower) && std::isfinite(yaw_upper)
            && yaw_lower <= expected_yaw && expected_yaw <= yaw_upper;
        if (!phase_valid || !envelope_finite)
        {
            state_.confidence = 0.0;
            ++low_confidence_steps_;
            return false;
        }
        previous_phase_ = std::clamp(phase, 0.0, 1.0);
        expected_height_ = expected_height;
        height_residual_ = state_.body_height_offset - expected_height_;
        expected_yaw_ = expected_yaw;
        yaw_residual_ = wrapAngle(state_.relative_yaw - expected_yaw_);
        const bool height_in_phase_envelope = state_.body_height_offset >= height_lower
            && state_.body_height_offset <= height_upper;
        const bool yaw_in_phase_envelope = state_.relative_yaw >= yaw_lower
            && state_.relative_yaw <= yaw_upper;
        height_in_phase_envelope_ = height_in_phase_envelope;
        yaw_in_phase_envelope_ = yaw_in_phase_envelope;
        const double target_confidence = base_domain_valid_ && height_in_phase_envelope
            && yaw_in_phase_envelope ? 1.0 : 0.0;
        state_.confidence = 0.92 * state_.confidence + 0.08 * target_confidence;
        if (state_.confidence < 0.35)
        {
            ++low_confidence_steps_;
        }
        else
        {
            low_confidence_steps_ = 0;
        }
        return true;
    }

    const State& state() const { return state_; }
    double phase() const { return previous_phase_; }
    double expectedHeight() const { return expected_height_; }
    double heightResidual() const { return height_residual_; }
    double expectedYaw() const { return expected_yaw_; }
    double yawResidual() const { return yaw_residual_; }
    bool heightInPhaseEnvelope() const { return height_in_phase_envelope_; }
    bool yawInPhaseEnvelope() const { return yaw_in_phase_envelope_; }
    bool baseDomainValid() const { return base_domain_valid_; }
    bool confidenceLowTooLong() const { return low_confidence_steps_ >= 8; }
    bool contactCriticalPhase() const { return previous_phase_ >= 0.15; }

private:
    static constexpr double kPi = 3.14159265358979323846;

    static double wrapAngle(double value)
    {
        while (value > kPi) value -= 2.0 * kPi;
        while (value < -kPi) value += 2.0 * kPi;
        return value;
    }

    static double yaw(const std::array<double, 4>& q)
    {
        return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                          1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
    }

    static double heightProxy(const std::array<double, 16>& q)
    {
        // Symmetric four-leg kinematic height proxy. Link lengths are used
        // only as a relative correction and do not encode platform geometry.
        constexpr double thigh = 0.24;
        constexpr double calf = 0.25;
        double height = 0.0;
        for (std::size_t leg = 0; leg < 4; ++leg)
        {
            const double hip_pitch = q[4 + leg];
            const double knee = q[8 + leg];
            height += -(thigh * std::cos(hip_pitch) + calf * std::cos(hip_pitch + knee));
        }
        return height / 4.0;
    }

    State state_{};
    double initial_yaw_ = 0.0;
    double initial_height_proxy_ = 0.0;
    double inertial_velocity_x_ = 0.0;
    double previous_phase_ = 0.0;
    int low_confidence_steps_ = 0;
    bool base_domain_valid_ = true;
    bool height_in_phase_envelope_ = true;
    bool yaw_in_phase_envelope_ = true;
    double expected_height_ = 0.0;
    double height_residual_ = 0.0;
    double expected_yaw_ = 0.0;
    double yaw_residual_ = 0.0;
};

}  // namespace highstep_virtual_terrain

#endif
