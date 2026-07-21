//
// Created by biao on 24-10-6.
//

#ifndef STATERL_H
#define STATERL_H

// #include <common/ObservationBuffer.h> // 【修改】不再需要原来的 ObservationBuffer
#include <rl_quadruped_adjustable_leg_controller/control/CtrlComponent.h>
#include <torch/script.h>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "controller_common/FSM/FSMState.h"
#include "rl_quadruped_adjustable_leg_controller/FSM/HighstepFixedMotionPolicy2Adapter.h"
#include "rl_quadruped_adjustable_leg_controller/FSM/HighstepTeacherVirtualTerrainAdapter.h"

struct CtrlComponent;

template <typename Functor>
void executeAndSleep(Functor f, const double frequency)
{
    using clock = std::chrono::high_resolution_clock;
    const auto start = clock::now();

    // Execute wrapped function
    f();

    // Compute desired duration rounded to clock decimation
    const std::chrono::duration<double> desiredDuration(1.0 / frequency);
    const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

    // Sleep
    const auto sleepTill = start + dt;
    std::this_thread::sleep_until(sleepTill);
}

inline void setThreadPriority(int priority, std::thread& thread)
{
    sched_param sched{};
    sched.sched_priority = priority;

    if (priority != 0)
    {
        if (pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &sched) != 0)
        {
            std::cerr << "WARNING: Failed to set threads priority (one possible reason could be "
                "that the user and the group permissions are not set properly.)"
                << std::endl;
        }
    }
}


template <typename T>
struct RobotCommand
{
    struct MotorCommand
    {
        std::vector<T> q = std::vector<T>(32, 0.0);
        std::vector<T> dq = std::vector<T>(32, 0.0);
        std::vector<T> tau = std::vector<T>(32, 0.0);
        std::vector<T> kp = std::vector<T>(32, 0.0);
        std::vector<T> kd = std::vector<T>(32, 0.0);
    } motor_command;
};

template <typename T>
struct RobotState
{
    struct IMU
    {
        std::vector<T> quaternion = {1.0, 0.0, 0.0, 0.0}; // w, x, y, z
        std::vector<T> gyroscope = {0.0, 0.0, 0.0};
        std::vector<T> accelerometer = {0.0, 0.0, 0.0};
    } imu;

    struct MotorState
    {
        std::vector<T> q = std::vector<T>(32, 0.0);
        std::vector<T> dq = std::vector<T>(32, 0.0);
        std::vector<T> ddq = std::vector<T>(32, 0.0);
        std::vector<T> tauEst = std::vector<T>(32, 0.0);
        std::vector<T> cur = std::vector<T>(32, 0.0);
    } motor_state;
};

struct Control
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    // double vel_x = 0.0;
    // double vel_y = 0.0;
    // double vel_yaw = 0.0;
    // double pos_x = 0.0;
    // double pos_y = 0.0;
    // double pos_z = 0.0;
    // double pos_roll = 0.0;
    // double pos_pitch = 0.0;
    // double pos_yaw = 0.0;
};

struct ModelParams
{
    std::string model_name;
    std::string framework;
    int decimation;
    int num_observations;
    std::vector<std::string> observations;
    std::vector<int> observations_history;
    double damping;
    double stiffness;
    // double action_scale;
    double hip_scale_reduction;
    std::vector<int> hip_scale_reduction_indices;
    int num_of_dofs;
    double lin_vel_scale;
    double ang_vel_scale;
    double dof_pos_scale;
    double dof_vel_scale;
    double clip_obs;
    torch::Tensor clip_actions_upper;
    torch::Tensor clip_actions_lower;
    torch::Tensor torque_limits;
    torch::Tensor rl_kd;
    torch::Tensor rl_kp;
    torch::Tensor commands_scale;
    torch::Tensor default_dof_pos;
    torch::Tensor action_scale;
    torch::Tensor output_dof_pos_lower;
    torch::Tensor output_dof_pos_upper;
    // Mandatory, independently reviewed hardware target envelope.  The optional
    // output_dof_pos_* policy envelope is always intersected with this envelope.
    torch::Tensor physical_dof_pos_lower;
    torch::Tensor physical_dof_pos_upper;
    bool rear_hip_guard_enabled = false;
    bool rear_hip_guard_policy2_only = true;
    double rear_hip_guard_min_cmd_x = 0.05;
    double rear_hip_guard_min_abs = 0.06;
    double rear_hip_guard_blend = 1.0;
    int rear_hip_guard_warning_interval = 200;
    bool policy2_fixed_motion_enabled = false;
    bool policy2_one_shot_enabled = false;
    // Logging-only: publish the existing raw-action/target contract trace for
    // an ordinary 570-D policy2 without changing inference or control output.
    bool policy2_contract_trace_enabled = false;
    double policy2_trigger_revolute_tolerance = 0.35;
    double policy2_trigger_box_tolerance = 0.015;
    double policy2_trigger_revolute_velocity_limit = 0.75;
    double policy2_trigger_box_velocity_limit = 0.04;
    bool teacher_virtual_terrain_enabled = false;
    std::string teacher_virtual_reference_model;
    double teacher_policy2_command_x_gain = 1.4400000572;
    double teacher_adapter_min_confidence = 0.35;
    bool diagnostic_soft_gates_log_only = false;
    std::string diagnostic_runtime_backend = "unknown";
};

struct Observations
{
    torch::Tensor lin_vel;
    torch::Tensor ang_vel;
    torch::Tensor gravity_vec;
    torch::Tensor commands;
    // torch::Tensor pose_commands;
    torch::Tensor base_quat;
    torch::Tensor dof_pos;
    torch::Tensor dof_vel;
    torch::Tensor actions;
};

class StateRL final : public FSMState
{
public:
    explicit StateRL(CtrlInterfaces& ctrl_interfaces,
                     CtrlComponent& ctrl_component,
                     const std::vector<double>& target_pos,
                     const std::vector<std::string>& controller_joint_names,
                     FSMStateName state_name = FSMStateName::RL,
                     const std::string& state_name_string = "rl",
                     const std::string& model_folder_parameter = "model_folder",
                     const std::string& model_name_key = "");

    ~StateRL() override;

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    // 【新增】获取当前帧的各个独立观测项
    std::map<std::string, torch::Tensor> computeCurrentObsTerms();

    torch::Tensor computeObservation();

    // torch::Tensor EulartoQuat(torch::Tensor euler);

    void loadYaml(const std::string& config_path);

    static torch::Tensor quatRotateInverse(const torch::Tensor& q, const torch::Tensor& v,
                                           const std::string& framework);

    /**
    * @brief Forward the RL model to get the action
    */
    torch::Tensor forward();

    void getState();

    void runModel(std::uint64_t command_epoch);

    void setCommand();

    bool initializeHoldCommand(const RobotState<double>& state_snapshot);

    bool validatePolicy2TriggerState(const RobotState<double>& state_snapshot) const;

    void publishDeploymentManifest();

    void publishCommandSafetyDebug(bool force = false);

    std::string buildDeploymentManifest() const;

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::string robot_pkg_ = "arcdog_adjustable_leg_description";
    std::string model_folder_ = "rl_policy";
    std::string model_folder_parameter_ = "model_folder";
    std::string model_name_key_;
    std::string resolved_model_name_key_;
    std::string config_file_path_;
    std::string resolved_model_path_;
    std::string config_fingerprint_;
    std::string model_fingerprint_;
    std::string deployment_manifest_;
    // Exact controller `joints` parameter, accepted only after canonical-order validation.
    std::vector<std::string> verified_joint_names_;

    bool enable_estimator_;
    std::shared_ptr<Estimator>& estimator_;

    // Parameters
    ModelParams params_;
    Observations obs_;
    Control control_;
    double init_pos_[16] = {};

    RobotState<double> robot_state_;
    RobotCommand<double> robot_command_;
    mutable std::mutex state_mutex_;
    mutable std::mutex command_mutex_;
    // Serializes inference against re-entry resets of obs_/obs_history_map_.
    mutable std::mutex inference_mutex_;

    // 【修改】废弃旧的 ObservationBuffer，使用按特征分类的独立历史缓冲区
    // std::shared_ptr<ObservationBuffer> history_obs_buf_;
    // torch::Tensor history_obs_;
    std::map<std::string, std::deque<torch::Tensor>> obs_history_map_;
    int history_length_ = 1;

    // rl module
    torch::jit::script::Module model_;
    torch::jit::script::Module teacher_virtual_reference_model_;
    bool use_rl_thread_ = true;
    std::thread rl_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> shutdown_thread_{false};
    std::atomic<bool> command_valid_{false};
    std::atomic<bool> inference_command_valid_{false};
    std::atomic<bool> deployment_fault_{false};
    std::atomic<std::uint64_t> command_epoch_{0};
    std::atomic<std::uint64_t> command_sequence_{0};
    std::atomic<std::uint64_t> inference_commit_count_{0};
    std::atomic<std::uint64_t> clamp_event_count_{0};
    std::atomic<float> last_clamp_max_delta_{0.0F};
    std::atomic<float> max_clamp_delta_{0.0F};
    std::atomic<double> first_inference_latency_ms_{-1.0};
    std::chrono::steady_clock::time_point enter_steady_time_{};
    std::uint64_t control_tick_counter_ = 0;
    int output_clamp_warning_counter_ = 0;
    int rear_hip_guard_warning_counter_ = 0;
    highstep_fixed_motion::Policy2Adapter policy2_fixed_motion_adapter_;
    highstep_fixed_motion::Policy2Adapter::CommandSample policy2_command_sample_;
    bool repeated_policy2_trigger_reported_ = false;
    highstep_virtual_terrain::Adapter teacher_virtual_adapter_;
    torch::Tensor teacher_fake_scan_ = torch::zeros({1, 102});
    double teacher_front_rear_delta_ = 0.04;
    double teacher_virtual_phase_ = 0.0;
    double teacher_previous_lookup_progress_ = 0.0;
    double teacher_previous_scan_max_delta_ = 0.0;
    bool teacher_positive_command_seen_ = false;
    bool diagnostic_mujoco_backend_verified_ = false;
    std::uint32_t teacher_soft_gate_event_mask_ = 0;
    std::uint64_t teacher_soft_gate_bypass_count_ = 0;
    bool teacher_reference_loaded_ = false;
    std::atomic<bool> deployment_fault_requires_passive_{true};

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr deployment_manifest_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_safety_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_clamp_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obs_dimension_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr scale_mismatch_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rear_hip_guard_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr policy2_contract_trace_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr teacher_virtual_adapter_trace_pub_;

    // output buffer
    torch::Tensor output_torques;
    torch::Tensor output_dof_pos_;
};

#endif //STATERL_H
