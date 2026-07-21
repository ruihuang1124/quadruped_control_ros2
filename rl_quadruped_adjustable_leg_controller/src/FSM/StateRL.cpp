
//
// Created by biao on 24-10-6.
//

#include "rl_quadruped_adjustable_leg_controller/FSM/StateRL.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
constexpr std::array<const char*, 16> kControllerJointOrder = {
    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
    "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
    "FL_box_joint", "FR_box_joint", "RL_box_joint", "RR_box_joint"};

std::string JsonEscape(const std::string& value)
{
    std::ostringstream stream;
    for (const char c : value)
    {
        switch (c)
        {
        case '\\': stream << "\\\\"; break;
        case '"': stream << "\\\""; break;
        case '\n': stream << "\\n"; break;
        case '\r': stream << "\\r"; break;
        case '\t': stream << "\\t"; break;
        default: stream << c; break;
        }
    }
    return stream.str();
}

// A dependency-free, deterministic file fingerprint.  It is intentionally
// labelled FNV-1a (not SHA256) so logs cannot be mistaken for a cryptographic
// digest.  The model/config absolute paths and byte counts are logged too.
std::string FingerprintFileFNV1a64(const std::string& path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
        throw std::runtime_error("Cannot open file for fingerprint: " + path);
    }

    constexpr std::uint64_t offset_basis = 14695981039346656037ULL;
    constexpr std::uint64_t prime = 1099511628211ULL;
    std::uint64_t hash = offset_basis;
    std::uint64_t byte_count = 0;
    std::array<char, 64 * 1024> buffer{};
    while (input)
    {
        input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const std::streamsize count = input.gcount();
        for (std::streamsize i = 0; i < count; ++i)
        {
            hash ^= static_cast<unsigned char>(buffer[static_cast<std::size_t>(i)]);
            hash *= prime;
        }
        byte_count += static_cast<std::uint64_t>(count);
    }

    std::ostringstream stream;
    stream << "fnv1a64:" << std::hex << std::setfill('0') << std::setw(16) << hash
           << std::dec << ":bytes:" << byte_count;
    return stream.str();
}

std::string TensorToJson(const torch::Tensor& tensor)
{
    const torch::Tensor flat = tensor.detach().to(torch::kCPU).to(torch::kDouble).contiguous().view({-1});
    std::ostringstream stream;
    stream << '[' << std::setprecision(10);
    for (std::int64_t i = 0; i < flat.numel(); ++i)
    {
        if (i != 0)
        {
            stream << ',';
        }
        stream << flat[i].item<double>();
    }
    stream << ']';
    return stream.str();
}

void RequireTensorSize(const torch::Tensor& tensor, const int expected, const std::string& name)
{
    if (tensor.numel() != expected)
    {
        throw std::invalid_argument(
            "YAML field '" + name + "' has " + std::to_string(tensor.numel())
            + " values; expected " + std::to_string(expected));
    }
    if (!torch::isfinite(tensor).all().item<bool>())
    {
        throw std::invalid_argument("YAML field '" + name + "' contains non-finite values");
    }
}
}  // namespace

template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node& node)
{
    std::vector<T> values;
    for (const auto& val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}

template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node& node, const std::string& framework, const int& rows,
                                  const int& cols)
{
    if (rows <= 0 || cols <= 0)
    {
        throw std::invalid_argument("YAML matrix rows and cols must both be positive");
    }

    std::vector<T> values;
    for (const auto& val : node)
    {
        values.push_back(val.as<T>());
    }

    const std::size_t row_count = static_cast<std::size_t>(rows);
    const std::size_t col_count = static_cast<std::size_t>(cols);
    if (row_count > std::numeric_limits<std::size_t>::max() / col_count)
    {
        throw std::invalid_argument("YAML matrix rows*cols overflows size_t");
    }
    const std::size_t expected_size = row_count * col_count;
    if (values.size() != expected_size)
    {
        throw std::invalid_argument(
            "YAML matrix has " + std::to_string(values.size()) + " values; expected exactly "
            + std::to_string(expected_size));
    }

    if (framework == "isaacsim")
    {
        std::vector<T> transposed_values(expected_size);
        for (std::size_t r = 0; r < row_count; ++r)
        {
            for (std::size_t c = 0; c < col_count; ++c)
            {
                transposed_values[c * row_count + r] = values[r * col_count + c];
            }
        }
        return transposed_values;
    }
    if (framework == "isaacgym")
    {
        return values;
    }
    throw std::invalid_argument("Unsupported framework: " + framework);
}

StateRL::StateRL(CtrlInterfaces& ctrl_interfaces,
                 CtrlComponent& ctrl_component,
                 const std::vector<double>& target_pos,
                 const std::vector<std::string>& controller_joint_names,
                 FSMStateName state_name,
                 const std::string& state_name_string,
                 const std::string& model_folder_parameter,
                 const std::string& model_name_key) :
    FSMState(state_name, state_name_string, ctrl_interfaces),
    node_(ctrl_component.node_),
    model_folder_parameter_(model_folder_parameter),
    model_name_key_(model_name_key),
    verified_joint_names_(controller_joint_names),
    enable_estimator_(ctrl_component.enable_estimator_),
    estimator_(ctrl_component.estimator_)
{
    if (verified_joint_names_.size() != kControllerJointOrder.size())
    {
        throw std::invalid_argument(
            "StateRL controller joints parameter has " + std::to_string(verified_joint_names_.size())
            + " entries; expected exactly " + std::to_string(kControllerJointOrder.size()));
    }
    for (std::size_t i = 0; i < kControllerJointOrder.size(); ++i)
    {
        if (verified_joint_names_[i] != kControllerJointOrder[i])
        {
            throw std::invalid_argument(
                "StateRL controller joint order mismatch at index " + std::to_string(i)
                + ": actual='" + verified_joint_names_[i] + "' expected='"
                + kControllerJointOrder[i] + "'. Refusing to map YAML gains/limits by position.");
        }
    }

    if (!node_->has_parameter("robot_pkg")) {
        node_->declare_parameter("robot_pkg", robot_pkg_);
    }
    if (!node_->has_parameter(model_folder_parameter_)) {
        node_->declare_parameter(model_folder_parameter_, model_folder_);
    }
    if (!node_->has_parameter("use_rl_thread")) {
        node_->declare_parameter("use_rl_thread", use_rl_thread_);
    }
    if (!node_->has_parameter("diagnostic_soft_gates_log_only")) {
        node_->declare_parameter("diagnostic_soft_gates_log_only", false);
    }
    if (!node_->has_parameter("diagnostic_runtime_backend")) {
        node_->declare_parameter("diagnostic_runtime_backend", "unknown");
    }
    robot_pkg_ = node_->get_parameter("robot_pkg").as_string();
    model_folder_ = node_->get_parameter(model_folder_parameter_).as_string();
    use_rl_thread_ = node_->get_parameter("use_rl_thread").as_bool();

    RCLCPP_INFO(node_->get_logger(), "Using robot model from %s", robot_pkg_.c_str());
    const std::string package_share_directory = ament_index_cpp::get_package_share_directory(robot_pkg_);
    const std::string model_path = package_share_directory + "/config/" + model_folder_;
    config_file_path_ = model_path + "/config.yaml";

    if (target_pos.size() != 16)
    {
        throw std::invalid_argument(
            "StateRL requires exactly 16 initial joint positions; received "
            + std::to_string(target_pos.size()));
    }
    for (int i = 0; i < 16; i++)
    {
        init_pos_[i] = target_pos[i];
    }

    // read params from yaml
    loadYaml(model_path);
    params_.diagnostic_soft_gates_log_only = state_name == FSMStateName::RLPOLICY2
        && node_->get_parameter("diagnostic_soft_gates_log_only").as_bool();
    params_.diagnostic_runtime_backend =
        node_->get_parameter("diagnostic_runtime_backend").as_string();
    if (params_.diagnostic_soft_gates_log_only)
    {
        const auto services = node_->get_service_names_and_types();
        const bool has_mujoco_arm_service = services.find("/highstep_contract_arm") != services.end();
        const bool has_mujoco_applied_command_publisher =
            node_->count_publishers("/mujoco_applied_actuators_cmds") > 0;
        diagnostic_mujoco_backend_verified_ = params_.diagnostic_runtime_backend == "mujoco"
            && has_mujoco_arm_service && has_mujoco_applied_command_publisher;
        if (!diagnostic_mujoco_backend_verified_)
        {
            throw std::runtime_error(
                "diagnostic_soft_gates_log_only refused: runtime is not positively verified as MuJoCo");
        }
        RCLCPP_WARN(
            node_->get_logger(),
            "MUJOCO-ONLY DIAGNOSTIC active: soft Teacher adapter gates are trace-only; hard protections remain active");
    }

    // 获取历史长度
    if (!params_.observations_history.empty())
    {
        history_length_ = params_.observations_history.size();
    }
    else
    {
        history_length_ = 1;
    }

    RCLCPP_INFO(
        node_->get_logger(), "Model loading for %s: key=%s folder_parameter=%s path=%s/%s",
        state_name_string.c_str(), resolved_model_name_key_.c_str(), model_folder_parameter_.c_str(),
        model_folder_.c_str(), params_.model_name.c_str());
    resolved_model_path_ = model_path + "/" + params_.model_name;
    
    // ==========================================
    // 【关键修复】：强制将模型映射到 CPU 上加载
    // 解决 'aten::empty_strided' with 'CUDA' backend 报错导致节点崩溃的问题
    // ==========================================
    try {
        model_ = torch::jit::load(resolved_model_path_, torch::kCPU);
        RCLCPP_INFO(node_->get_logger(), "✅ Model successfully loaded on CPU.");
    } catch (const c10::Error& e) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to load model: %s", e.what());
        throw std::runtime_error("StateRL cannot safely start because the selected policy failed to load");
    }
    if (params_.teacher_virtual_terrain_enabled)
    {
        const std::string reference_path = model_path + "/" + params_.teacher_virtual_reference_model;
        try
        {
            teacher_virtual_reference_model_ = torch::jit::load(reference_path, torch::kCPU);
            teacher_reference_loaded_ = true;
            RCLCPP_INFO(
                node_->get_logger(),
                "Loaded frozen Teacher virtual-terrain reference '%s' (runtime inputs: adapter state + q only)",
                reference_path.c_str());
        }
        catch (const c10::Error& error)
        {
            RCLCPP_ERROR(node_->get_logger(), "Cannot load frozen virtual-terrain reference: %s", error.what());
            throw std::runtime_error("Teacher virtual-terrain reference failed to load");
        }
    }

    config_fingerprint_ = FingerprintFileFNV1a64(config_file_path_);
    model_fingerprint_ = FingerprintFileFNV1a64(resolved_model_path_);
    deployment_manifest_ = buildDeploymentManifest();

    rclcpp::QoS manifest_qos(1);
    // Both policy StateRL objects publish this topic. Volatile + periodic
    // republish avoids a late recorder receiving two cached transient manifests
    // (one stale policy1 and one current policy2).
    manifest_qos.reliable().durability_volatile();
    deployment_manifest_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/rl_deployment_manifest", manifest_qos);
    command_safety_debug_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/rl_command_safety_debug", rclcpp::SensorDataQoS());
    target_clamp_debug_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/rl_target_clamp_debug", rclcpp::SensorDataQoS());
    obs_dimension_debug_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/rl_obs_dimension_debug", rclcpp::SensorDataQoS());
    scale_mismatch_debug_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/rl_scale_mismatch_debug", rclcpp::SensorDataQoS());
    rear_hip_guard_debug_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/rl_highstep_rear_hip_guard_debug", rclcpp::SensorDataQoS());
    policy2_contract_trace_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/rl_policy2_contract_trace", rclcpp::SensorDataQoS());
    teacher_virtual_adapter_trace_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/rl_teacher_virtual_adapter_trace", rclcpp::SensorDataQoS());

    RCLCPP_INFO(
        node_->get_logger(),
        "RL deployment identity state='%s' model_key='%s' model='%s' model_fingerprint='%s' "
        "config='%s' config_fingerprint='%s' build='%s %s'",
        state_name_string.c_str(), resolved_model_name_key_.c_str(), resolved_model_path_.c_str(),
        model_fingerprint_.c_str(), config_file_path_.c_str(), config_fingerprint_.c_str(),
        __DATE__, __TIME__);
    for (int i = 0; i < params_.num_of_dofs; ++i)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "RL deployment contract state='%s' joint[%d]=%s kp=%.8f kd=%.8f action_scale=%.8f "
            "physical_target=[%.8f,%.8f] default=%.8f",
            state_name_string.c_str(), i, verified_joint_names_[static_cast<std::size_t>(i)].c_str(),
            params_.rl_kp[0][i].item<double>(), params_.rl_kd[0][i].item<double>(),
            params_.action_scale[0][i].item<double>(),
            params_.physical_dof_pos_lower[0][i].item<double>(),
            params_.physical_dof_pos_upper[0][i].item<double>(),
            params_.default_dof_pos[0][i].item<double>());
    }

    // for (const auto &param: model_.parameters()) {
    //     std::cout << "Parameter dtype: " << param.dtype() << std::endl;
    // }


    if (use_rl_thread_)
    {
        rl_thread_ = std::thread([this]{
            while (!shutdown_thread_.load(std::memory_order_acquire))
            {
                std::uint64_t inference_epoch = 0;
                try
                {
                    executeAndSleep(
                        [&]
                        {
                            if (running_.load(std::memory_order_acquire))
                            {
                                inference_epoch = command_epoch_.load(std::memory_order_acquire);
                                runModel(inference_epoch);
                            }
                        },
                        ctrl_interfaces_.frequency_ / params_.decimation);
                }
                catch (const std::exception& e)
                {
                    bool current_epoch_failed = false;
                    std::uint64_t current_epoch = 0;
                    {
                        std::lock_guard<std::mutex> lock(command_mutex_);
                        current_epoch = command_epoch_.load(std::memory_order_acquire);
                        current_epoch_failed = inference_epoch != 0 && inference_epoch == current_epoch;
                        if (current_epoch_failed)
                        {
                            running_.store(false, std::memory_order_release);
                            deployment_fault_.store(true, std::memory_order_release);
                        }
                    }
                    if (current_epoch_failed)
                    {
                        RCLCPP_ERROR(
                            node_->get_logger(),
                            "RL inference stopped; current hold command remains latched and FSM will request passive: %s",
                            e.what());
                    }
                    else
                    {
                        RCLCPP_WARN(
                            node_->get_logger(),
                            "Ignoring exception from stale RL inference epoch=%llu current_epoch=%llu: %s",
                            static_cast<unsigned long long>(inference_epoch),
                            static_cast<unsigned long long>(current_epoch),
                            e.what());
                    }
                }
            }
        });
        setThreadPriority(60, rl_thread_);
    }
}

StateRL::~StateRL()
{
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        running_.store(false, std::memory_order_release);
        command_valid_.store(false, std::memory_order_release);
        command_epoch_.fetch_add(1, std::memory_order_acq_rel);
    }
    shutdown_thread_.store(true, std::memory_order_release);
    if (rl_thread_.joinable())
    {
        rl_thread_.join();
    }
}

void StateRL::enter()
{
    std::uint64_t epoch = 0;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        running_.store(false, std::memory_order_release);
        epoch = command_epoch_.fetch_add(1, std::memory_order_acq_rel) + 1;
        command_valid_.store(false, std::memory_order_release);
        inference_command_valid_.store(false, std::memory_order_release);
    }

    RCLCPP_INFO(
        node_->get_logger(), "Entering %s epoch=%llu model_key=%s model=%s",
        state_name_string.c_str(), static_cast<unsigned long long>(epoch),
        resolved_model_name_key_.c_str(), resolved_model_path_.c_str());

    // A previous inference from this same StateRL instance may still be
    // unwinding after exit(). Serialize it against obs/history reinitialization.
    std::unique_lock<std::mutex> inference_lock(inference_mutex_);
    // Reset per-entry metrics only after the old inference has completely
    // stopped; otherwise its post-commit clamp accounting could leak into this
    // entry even though its epoch can no longer publish a command.
    deployment_fault_.store(false, std::memory_order_release);
    first_inference_latency_ms_.store(-1.0, std::memory_order_release);
    inference_commit_count_.store(0, std::memory_order_release);
    clamp_event_count_.store(0, std::memory_order_release);
    last_clamp_max_delta_.store(0.0F, std::memory_order_release);
    max_clamp_delta_.store(0.0F, std::memory_order_release);
    control_tick_counter_ = 0;
    policy2_fixed_motion_adapter_.reset();
    repeated_policy2_trigger_reported_ = false;
    enter_steady_time_ = std::chrono::steady_clock::now();

    // Init observations
    obs_.lin_vel = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.ang_vel = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    obs_.commands = torch::tensor({{0.0, 0.0, 0.0}});
    // obs_.pose_commands = torch::tensor({{0.55, 0.0, 0.35, 1.0, 0.0, 0.0, 0.0}});
    obs_.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}});
    obs_.dof_pos = params_.default_dof_pos;
    obs_.dof_vel = torch::zeros({1, params_.num_of_dofs});
    obs_.actions = torch::zeros({1, params_.num_of_dofs});

    // Init output
    output_torques = torch::zeros({1, params_.num_of_dofs});
    output_dof_pos_ = params_.default_dof_pos;

    // 在初始化历史缓冲区前，先获取一次真实状态，避免全填 0 导致突变
    getState();
    RobotState<double> state_snapshot;
    Control control_snapshot;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_snapshot = robot_state_;
        control_snapshot = control_;
    }

    if (params_.teacher_virtual_terrain_enabled)
    {
        std::array<double, 4> quaternion{};
        std::array<double, 16> joint_position{};
        std::copy_n(state_snapshot.imu.quaternion.begin(), 4, quaternion.begin());
        std::copy_n(state_snapshot.motor_state.q.begin(), 16, joint_position.begin());
        teacher_virtual_adapter_.reset(quaternion, joint_position);
        teacher_fake_scan_ = torch::zeros({1, 102}, torch::kFloat32);
        teacher_front_rear_delta_ = 0.04;
        teacher_virtual_phase_ = 0.0;
        teacher_previous_lookup_progress_ = 0.0;
        teacher_previous_scan_max_delta_ = 0.0;
        teacher_positive_command_seen_ = false;
        teacher_soft_gate_event_mask_ = 0;
        teacher_soft_gate_bypass_count_ = 0;
        // Unknown inference failures must take the conservative route.  Known
        // pre-contact soft-gate failures explicitly downgrade this latch to
        // Fixed Stand at the point where their phase is available.
        deployment_fault_requires_passive_.store(true, std::memory_order_release);
    }

    if (params_.policy2_fixed_motion_enabled)
    {
        if (!params_.policy2_one_shot_enabled || !validatePolicy2TriggerState(state_snapshot)
            || !policy2_fixed_motion_adapter_.trigger())
        {
            deployment_fault_.store(true, std::memory_order_release);
            RCLCPP_ERROR(
                node_->get_logger(),
                "Refusing policy2 one-shot: trigger is not from an allowed converged Fixed Stand state");
            publishDeploymentManifest();
            publishCommandSafetyDebug(true);
            return;
        }
        // Consume the transition command. A later RB+LB+Y while this StateRL
        // instance is running is a repeated trigger and is rejected below.
        ctrl_interfaces_.control_inputs_.command = -1;
        RCLCPP_WARN(
            node_->get_logger(),
            "Policy2 one-shot accepted: executing 138 inference steps (21 zero + 59 vx=%.10f + 58 zero)",
            highstep_fixed_motion::Policy2Adapter::kFullPushVx);
    }

    // Publish a complete, non-zero-gain current-position hold before enabling
    // the inference thread.  This closes the 59--70 ms all-zero command window
    // observed in both 2026-07-07 bags.
    if (!initializeHoldCommand(state_snapshot))
    {
        deployment_fault_.store(true, std::memory_order_release);
        RCLCPP_ERROR(
            node_->get_logger(),
            "Refusing to enter %s: a finite in-bounds current-position hold could not be formed",
            state_name_string.c_str());
        publishDeploymentManifest();
        publishCommandSafetyDebug(true);
        return;
    }

    if (enable_estimator_) {
        obs_.lin_vel = torch::from_blob(estimator_->getVelocity().data(), {3}, torch::kDouble).clone().to(torch::kFloat).unsqueeze(0);
    }
    obs_.ang_vel = torch::tensor(state_snapshot.imu.gyroscope).unsqueeze(0);
    obs_.commands = torch::tensor({{control_snapshot.x, control_snapshot.y, control_snapshot.yaw}});
    obs_.base_quat = torch::tensor(state_snapshot.imu.quaternion).unsqueeze(0);
    obs_.dof_pos = torch::tensor(state_snapshot.motor_state.q).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);
    obs_.dof_vel = torch::tensor(state_snapshot.motor_state.dq).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);

    // 初始化按特征分类的历史缓冲区
    obs_history_map_.clear();
    std::map<std::string, torch::Tensor> init_terms = computeCurrentObsTerms();
    for (const auto& obs_name : params_.observations) {
        for (int i = 0; i < history_length_; ++i) {
            obs_history_map_[obs_name].push_back(init_terms[obs_name]);
        }
    }

    publishDeploymentManifest();
    publishCommandSafetyDebug(true);
    {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        if (epoch != command_epoch_.load(std::memory_order_acquire))
        {
            command_valid_.store(false, std::memory_order_release);
            inference_command_valid_.store(false, std::memory_order_release);
            RCLCPP_WARN(
                node_->get_logger(),
                "Cancelling stale StateRL enter epoch=%llu current_epoch=%llu",
                static_cast<unsigned long long>(epoch),
                static_cast<unsigned long long>(command_epoch_.load(std::memory_order_acquire)));
            publishCommandSafetyDebug(true);
            return;
        }
        running_.store(true, std::memory_order_release);
    }
    inference_lock.unlock();
}

void StateRL::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    getState();
    if (!use_rl_thread_)
    {
        try
        {
            runModel(command_epoch_.load(std::memory_order_acquire));
        }
        catch (const std::exception& error)
        {
            running_.store(false, std::memory_order_release);
            deployment_fault_.store(true, std::memory_order_release);
            RCLCPP_ERROR(
                node_->get_logger(),
                "Synchronous RL inference failed; retaining the last valid hold/command and requesting passive: %s",
                error.what());
        }
    }
    setCommand();
}

void StateRL::exit()
{
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        running_.store(false, std::memory_order_release);
        command_epoch_.fetch_add(1, std::memory_order_acq_rel);
        command_valid_.store(false, std::memory_order_release);
        inference_command_valid_.store(false, std::memory_order_release);
    }
    // Wait for any inference that passed the running check before exit(). No
    // new inference can start after running=false, so interfaces/lifecycle
    // owners may safely deactivate once this barrier returns.
    {
        std::lock_guard<std::mutex> inference_lock(inference_mutex_);
    }
    publishCommandSafetyDebug(true);
}

FSMStateName StateRL::checkChange()
{
    if (deployment_fault_.load(std::memory_order_acquire))
    {
        if (params_.teacher_virtual_terrain_enabled
            && !deployment_fault_requires_passive_.load(std::memory_order_acquire))
        {
            return FSMStateName::FIXEDSTANDADJUSTABLELEG;
        }
        return FSMStateName::PASSIVEADJUSTABLELEG;
    }
    if (enable_estimator_ and !estimator_->safety())
    {
        return FSMStateName::PASSIVEADJUSTABLELEG;
    }
    if (params_.policy2_fixed_motion_enabled && ctrl_interfaces_.control_inputs_.command == 4)
    {
        if (!repeated_policy2_trigger_reported_)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Rejected repeated policy2 trigger; return to Fixed Stand before starting another one-shot");
            repeated_policy2_trigger_reported_ = true;
        }
        ctrl_interfaces_.control_inputs_.command = -1;
        return state_name;
    }
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 0:
        return FSMStateName::PASSIVEADJUSTABLELEG;
    case 1:
        return FSMStateName::FIXEDDOWNADJUSTABLELEG;
    case 2:
        return FSMStateName::FIXEDSTANDADJUSTABLELEG;
    default:
        return state_name;
    }
}

// 单独计算当前帧的各个观测项
std::map<std::string, torch::Tensor> StateRL::computeCurrentObsTerms()
{
    std::map<std::string, torch::Tensor> terms;
    for (const std::string& observation : params_.observations)
    {
        if (observation == "lin_vel")
        {
            terms["lin_vel"] = obs_.lin_vel * params_.lin_vel_scale;
        }
        else if (observation == "ang_vel")
        {
            // obs_list.push_back(
            //     quatRotateInverse(obs_.base_quat, obs_.ang_vel, params_.framework) * params_.ang_vel_scale);
            terms["ang_vel"] = obs_.ang_vel * params_.ang_vel_scale;
        }
        else if (observation == "gravity_vec")
        {
            terms["gravity_vec"] = quatRotateInverse(obs_.base_quat, obs_.gravity_vec, params_.framework);
        }
        else if (observation == "commands")
        {
            terms["commands"] = obs_.commands * params_.commands_scale;
        // else if (observation == "vel_commands")
        // {
        //     obs_list.push_back(obs_.commands);
        // }
        // else if (observation == "pose_commands")
        // {
        //     obs_list.push_back(obs_.pose_commands); // scale TODO.
        }
        else if (observation == "dof_pos")
        {
            terms["dof_pos"] = (obs_.dof_pos - params_.default_dof_pos) * params_.dof_pos_scale;
        }
        else if (observation == "dof_vel")
        {
            terms["dof_vel"] = obs_.dof_vel * params_.dof_vel_scale;
        }
        else if (observation == "actions")
        {
            terms["actions"] = obs_.actions;
        }
    }
    return terms;
}

// 按 Isaac Lab 的排布方式拼接历史观测
torch::Tensor StateRL::computeObservation()
{
    std::map<std::string, torch::Tensor> current_terms = computeCurrentObsTerms();
    std::vector<torch::Tensor> final_obs_list;

    // 遍历每一个特征（例如：先处理ang_vel，再处理gravity_vec...）
    for (const std::string& obs_name : params_.observations)
    {
        if (history_length_ > 1) {
            // 【关键修复】：Isaac Lab 的时间顺序是 [t-9, t-8, ... t-1, t]
            // 所以必须把最新帧放在末尾 (push_back)，把最老帧从头部移除 (pop_front)
            obs_history_map_[obs_name].push_back(current_terms[obs_name]);
            obs_history_map_[obs_name].pop_front();
        } else {
            obs_history_map_[obs_name][0] = current_terms[obs_name];
        }

        // 把这个特征的 10 帧历史拼接起来
        std::vector<torch::Tensor> term_history_vec(obs_history_map_[obs_name].begin(), obs_history_map_[obs_name].end());
        torch::Tensor term_flat = torch::cat(term_history_vec, 1);
        
        // 加入最终的列表
        final_obs_list.push_back(term_flat);
    }

    // 最后把所有特征的历史块拼接起来，完美匹配 Isaac Lab 的内存排布！
    const torch::Tensor obs = torch::cat(final_obs_list, 1).to(torch::kFloat32);

    // std::cout << "Observation: " << obs << std::endl;
    torch::Tensor clamped_obs = clamp(obs, -params_.clip_obs, params_.clip_obs);
    return clamped_obs;
}

void StateRL::loadYaml(const std::string& config_path)
{
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path + "/config.yaml");
    }
    catch ([[maybe_unused]] YAML::BadFile& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("StateRL"), "The file '%s' does not exist", config_path.c_str());
        throw std::runtime_error("StateRL configuration file is missing: " + config_path + "/config.yaml");
    }

    const std::string model_name_key = model_name_key_.empty() ? "model_name" : model_name_key_;
    resolved_model_name_key_ = model_name_key;
    if (config[model_name_key] && config[model_name_key].IsScalar())
    {
        params_.model_name = config[model_name_key].as<std::string>();
    }
    else
    {
        // Never silently run policy1 when policy2 was requested.
        throw std::invalid_argument(
            "Required YAML policy key '" + model_name_key + "' is missing or is not a scalar");
    }
    if (params_.model_name.empty())
    {
        throw std::invalid_argument("Selected policy filename is empty for key '" + model_name_key + "'");
    }
    params_.framework = config["framework"].as<std::string>();
    const int rows = config["rows"].as<int>();
    const int cols = config["cols"].as<int>();
    if (rows <= 0 || cols <= 0)
    {
        throw std::invalid_argument("YAML rows and cols must both be positive");
    }
    if (rows != 4 || cols != 4)
    {
        throw std::invalid_argument(
            "StateRL YAML transpose contract requires rows=4 legs and cols=4 joint types exactly");
    }
    if (config["observations_history"].IsNull())
    {
        params_.observations_history = {};
    }
    else
    {
        params_.observations_history = ReadVectorFromYaml<int>(config["observations_history"]);
    }
    params_.decimation = config["decimation"].as<int>();
    if (params_.decimation <= 0)
    {
        throw std::invalid_argument("YAML decimation must be positive");
    }
    params_.num_observations = config["num_observations"].as<int>();
    params_.observations = ReadVectorFromYaml<std::string>(config["observations"]);
    params_.clip_obs = config["clip_obs"].as<double>();
    const bool has_clip_lower = config["clip_actions_lower"] && config["clip_actions_lower"].IsSequence();
    const bool has_clip_upper = config["clip_actions_upper"] && config["clip_actions_upper"].IsSequence();
    if (!has_clip_lower && !has_clip_upper)
    {
        params_.clip_actions_upper = torch::tensor({}).view({1, -1});
        params_.clip_actions_lower = torch::tensor({}).view({1, -1});
    }
    else if (has_clip_lower && has_clip_upper)
    {
        params_.clip_actions_upper = torch::tensor(
            ReadVectorFromYaml<double>(config["clip_actions_upper"], params_.framework, rows, cols)).view({1, -1});
        params_.clip_actions_lower = torch::tensor(
            ReadVectorFromYaml<double>(config["clip_actions_lower"], params_.framework, rows, cols)).view({1, -1});
    }
    else
    {
        throw std::invalid_argument("clip_actions_lower and clip_actions_upper must be provided together");
    }
    // params_.action_scale = config["action_scale"].as<double>();
    params_.hip_scale_reduction = config["hip_scale_reduction"].as<double>();
    params_.hip_scale_reduction_indices = ReadVectorFromYaml<int>(config["hip_scale_reduction_indices"]);
    params_.num_of_dofs = config["num_of_dofs"].as<int>();
    params_.action_scale = torch::tensor(
        ReadVectorFromYaml<double>(config["action_scale"], params_.framework, rows, cols)
    ).view({1, -1}).to(torch::kFloat);
    params_.lin_vel_scale = config["lin_vel_scale"].as<double>();
    params_.ang_vel_scale = config["ang_vel_scale"].as<double>();
    params_.dof_pos_scale = config["dof_pos_scale"].as<double>();
    params_.dof_vel_scale = config["dof_vel_scale"].as<double>();
    params_.commands_scale = torch::tensor(ReadVectorFromYaml<double>(config["commands_scale"])).view({1, -1});
    // params_.commands_scale = torch::tensor({params_.lin_vel_scale, params_.lin_vel_scale, params_.ang_vel_scale});
    params_.rl_kp = torch::tensor(ReadVectorFromYaml<double>(config["rl_kp"], params_.framework, rows, cols)).view({
        1, -1
    });
    params_.rl_kd = torch::tensor(ReadVectorFromYaml<double>(config["rl_kd"], params_.framework, rows, cols)).view({
        1, -1
    });
    params_.torque_limits = torch::tensor(
        ReadVectorFromYaml<double>(config["torque_limits"], params_.framework, rows, cols)).view({1, -1});

    // ==========================================
    // 🌟【核心修复】：安全地读取 default_dof_pos
    // 修复了 yaml-cpp 迭代空指针导致的 Segmentation fault。
    // 如果 config.yaml 中没有写 default_dof_pos，代码会安全地回退到 init_pos_。
    // ==========================================
    if (config["default_dof_pos"] && config["default_dof_pos"].IsSequence()) {
        params_.default_dof_pos = torch::tensor(
            ReadVectorFromYaml<double>(config["default_dof_pos"], params_.framework, rows, cols)).view({1, -1}).to(torch::kFloat);
        RCLCPP_INFO(rclcpp::get_logger("StateRL"), "✅ Successfully loaded default_dof_pos from YAML.");
    } else {
        RCLCPP_WARN(rclcpp::get_logger("StateRL"), "⚠️ YAML 中缺少 default_dof_pos！安全回退使用 init_pos_。请务必在 config.yaml 中加上训练时的 default_dof_pos，否则 VAE 会推断出错误的 Latent！");
        params_.default_dof_pos = torch::from_blob(init_pos_, {16}, torch::kDouble).clone().to(torch::kFloat).unsqueeze(0);
    }

    // params_.default_dof_pos = torch::tensor(
    //     ReadVectorFromYaml<double>(config["default_dof_pos"], params_.framework, rows, cols)).view({1, -1});
    if (config["output_dof_pos_lower"] && config["output_dof_pos_upper"]
        && config["output_dof_pos_lower"].IsSequence() && config["output_dof_pos_upper"].IsSequence())
    {
        params_.output_dof_pos_lower = torch::tensor(
            ReadVectorFromYaml<double>(config["output_dof_pos_lower"], params_.framework, rows, cols)).view({1, -1}).to(torch::kFloat);
        params_.output_dof_pos_upper = torch::tensor(
            ReadVectorFromYaml<double>(config["output_dof_pos_upper"], params_.framework, rows, cols)).view({1, -1}).to(torch::kFloat);
        RCLCPP_INFO(rclcpp::get_logger("StateRL"), "✅ Loaded final output_dof_pos clamp from YAML.");
    }
    else
    {
        params_.output_dof_pos_lower = torch::tensor({}).view({1, -1});
        params_.output_dof_pos_upper = torch::tensor({}).view({1, -1});
        RCLCPP_WARN(
            rclcpp::get_logger("StateRL"),
            "No optional output_dof_pos policy envelope configured; mandatory physical limits still apply.");
    }

    // This envelope is deliberately a separate mandatory field. Falling back
    // to +/-60 or inferred limits would recreate the 0707 target-overrun bug.
    if (config["physical_dof_pos_lower"] && config["physical_dof_pos_upper"]
        && config["physical_dof_pos_lower"].IsSequence() && config["physical_dof_pos_upper"].IsSequence())
    {
        params_.physical_dof_pos_lower = torch::tensor(
            ReadVectorFromYaml<double>(config["physical_dof_pos_lower"], params_.framework, rows, cols))
            .view({1, -1}).to(torch::kFloat);
        params_.physical_dof_pos_upper = torch::tensor(
            ReadVectorFromYaml<double>(config["physical_dof_pos_upper"], params_.framework, rows, cols))
            .view({1, -1}).to(torch::kFloat);
    }
    else
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("StateRL"),
            "Missing mandatory physical_dof_pos_lower/upper. Refusing to arm RL; do not infer real-robot limits.");
        throw std::invalid_argument(
            "physical_dof_pos_lower and physical_dof_pos_upper are mandatory deployment safety fields");
    }
    params_.rear_hip_guard_enabled = config["rear_hip_guard_enabled"]
        ? config["rear_hip_guard_enabled"].as<bool>()
        : false;
    params_.rear_hip_guard_policy2_only = config["rear_hip_guard_policy2_only"]
        ? config["rear_hip_guard_policy2_only"].as<bool>()
        : true;
    params_.rear_hip_guard_min_cmd_x = config["rear_hip_guard_min_cmd_x"]
        ? config["rear_hip_guard_min_cmd_x"].as<double>()
        : 0.05;
    params_.rear_hip_guard_min_abs = config["rear_hip_guard_min_abs"]
        ? config["rear_hip_guard_min_abs"].as<double>()
        : 0.06;
    params_.rear_hip_guard_blend = config["rear_hip_guard_blend"]
        ? config["rear_hip_guard_blend"].as<double>()
        : 1.0;
    params_.rear_hip_guard_warning_interval = config["rear_hip_guard_warning_interval"]
        ? config["rear_hip_guard_warning_interval"].as<int>()
        : 200;
    const bool configured_policy2_fixed_motion = config["policy2_fixed_motion_enabled"]
        ? config["policy2_fixed_motion_enabled"].as<bool>()
        : false;
    params_.policy2_fixed_motion_enabled =
        state_name == FSMStateName::RLPOLICY2 && configured_policy2_fixed_motion;
    params_.policy2_one_shot_enabled = params_.policy2_fixed_motion_enabled
        && config["policy2_one_shot_enabled"]
        && config["policy2_one_shot_enabled"].as<bool>();
    const bool configured_policy2_contract_trace = config["policy2_contract_trace_enabled"]
        ? config["policy2_contract_trace_enabled"].as<bool>()
        : false;
    params_.policy2_contract_trace_enabled =
        state_name == FSMStateName::RLPOLICY2 && configured_policy2_contract_trace;
    if (params_.policy2_fixed_motion_enabled && !params_.policy2_one_shot_enabled)
    {
        throw std::invalid_argument("fixed-motion policy2 requires policy2_one_shot_enabled=true");
    }
    if (params_.policy2_fixed_motion_enabled)
    {
        params_.policy2_trigger_revolute_tolerance = config["policy2_trigger_revolute_tolerance"].as<double>();
        params_.policy2_trigger_box_tolerance = config["policy2_trigger_box_tolerance"].as<double>();
        params_.policy2_trigger_revolute_velocity_limit = config["policy2_trigger_revolute_velocity_limit"].as<double>();
        params_.policy2_trigger_box_velocity_limit = config["policy2_trigger_box_velocity_limit"].as<double>();
        if (params_.policy2_trigger_revolute_tolerance <= 0.0
            || params_.policy2_trigger_box_tolerance <= 0.0
            || params_.policy2_trigger_revolute_velocity_limit <= 0.0
            || params_.policy2_trigger_box_velocity_limit <= 0.0)
        {
            throw std::invalid_argument("policy2 one-shot trigger tolerances must all be positive");
        }
    }
    const bool configured_teacher_virtual_terrain = config["teacher_virtual_terrain_enabled"]
        ? config["teacher_virtual_terrain_enabled"].as<bool>()
        : false;
    params_.teacher_virtual_terrain_enabled =
        state_name == FSMStateName::RLPOLICY2 && configured_teacher_virtual_terrain;
    if (params_.teacher_virtual_terrain_enabled)
    {
        if (params_.policy2_fixed_motion_enabled)
        {
            throw std::invalid_argument("Teacher virtual terrain and fixed-motion one-shot are mutually exclusive");
        }
        if (!config["teacher_virtual_reference_model"] || !config["teacher_virtual_reference_model"].IsScalar())
        {
            throw std::invalid_argument("teacher_virtual_reference_model is mandatory when adapter is enabled");
        }
        params_.teacher_virtual_reference_model = config["teacher_virtual_reference_model"].as<std::string>();
        params_.teacher_policy2_command_x_gain = config["teacher_policy2_command_x_gain"]
            ? config["teacher_policy2_command_x_gain"].as<double>() : 1.4400000572;
        params_.teacher_adapter_min_confidence = config["teacher_adapter_min_confidence"]
            ? config["teacher_adapter_min_confidence"].as<double>() : 0.35;
        if (params_.teacher_virtual_reference_model.empty()
            || !(params_.teacher_policy2_command_x_gain > 0.0)
            || !(params_.teacher_adapter_min_confidence > 0.0 && params_.teacher_adapter_min_confidence < 1.0))
        {
            throw std::invalid_argument("invalid Teacher virtual-terrain adapter configuration");
        }
    }
    if (params_.rear_hip_guard_enabled)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("StateRL"),
            "Rear hip guard is ENABLED for highstep validation. min_abs=%.4f blend=%.3f min_cmd_x=%.3f policy2_only=%s",
            params_.rear_hip_guard_min_abs,
            params_.rear_hip_guard_blend,
            params_.rear_hip_guard_min_cmd_x,
            params_.rear_hip_guard_policy2_only ? "true" : "false");
    }

    const std::size_t yaml_matrix_size =
        static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
    if (params_.num_of_dofs != 16
        || yaml_matrix_size != static_cast<std::size_t>(params_.num_of_dofs))
    {
        throw std::invalid_argument("StateRL adjustable-leg deployment requires rows*cols=num_of_dofs=16");
    }
    RequireTensorSize(params_.action_scale, params_.num_of_dofs, "action_scale");
    RequireTensorSize(params_.rl_kp, params_.num_of_dofs, "rl_kp");
    RequireTensorSize(params_.rl_kd, params_.num_of_dofs, "rl_kd");
    RequireTensorSize(params_.torque_limits, params_.num_of_dofs, "torque_limits");
    RequireTensorSize(params_.default_dof_pos, params_.num_of_dofs, "default_dof_pos");
    RequireTensorSize(params_.physical_dof_pos_lower, params_.num_of_dofs, "physical_dof_pos_lower");
    RequireTensorSize(params_.physical_dof_pos_upper, params_.num_of_dofs, "physical_dof_pos_upper");
    if (params_.clip_actions_lower.numel() != 0)
    {
        RequireTensorSize(params_.clip_actions_lower, params_.num_of_dofs, "clip_actions_lower");
        RequireTensorSize(params_.clip_actions_upper, params_.num_of_dofs, "clip_actions_upper");
        if (!(params_.clip_actions_lower <= params_.clip_actions_upper).all().item<bool>())
        {
            throw std::invalid_argument("clip_actions_lower exceeds clip_actions_upper");
        }
    }
    if (params_.output_dof_pos_lower.numel() != 0)
    {
        RequireTensorSize(params_.output_dof_pos_lower, params_.num_of_dofs, "output_dof_pos_lower");
        RequireTensorSize(params_.output_dof_pos_upper, params_.num_of_dofs, "output_dof_pos_upper");
    }
    if (!(params_.physical_dof_pos_lower < params_.physical_dof_pos_upper).all().item<bool>())
    {
        throw std::invalid_argument("Each physical_dof_pos_lower must be strictly below its upper bound");
    }
    if ((params_.rl_kp <= 0).any().item<bool>() || (params_.rl_kd < 0).any().item<bool>())
    {
        throw std::invalid_argument("RL kp must be positive and kd must be non-negative");
    }
    if ((params_.action_scale <= 0).any().item<bool>())
    {
        throw std::invalid_argument("action_scale must be finite and strictly positive");
    }
    if ((params_.torque_limits < 0).any().item<bool>())
    {
        throw std::invalid_argument("torque_limits must be finite and non-negative");
    }
    if ((params_.default_dof_pos < params_.physical_dof_pos_lower).any().item<bool>()
        || (params_.default_dof_pos > params_.physical_dof_pos_upper).any().item<bool>())
    {
        throw std::invalid_argument("default_dof_pos lies outside the mandatory physical target envelope");
    }
    if (params_.output_dof_pos_lower.numel() != 0)
    {
        const torch::Tensor effective_lower = torch::maximum(
            params_.output_dof_pos_lower, params_.physical_dof_pos_lower);
        const torch::Tensor effective_upper = torch::minimum(
            params_.output_dof_pos_upper, params_.physical_dof_pos_upper);
        if (!(effective_lower < effective_upper).all().item<bool>())
        {
            throw std::invalid_argument("Optional output envelope has an empty intersection with physical limits");
        }
    }

    RCLCPP_WARN(
        rclcpp::get_logger("StateRL"),
        "Mandatory physical target clamp armed. Verify these YAML values against the exact onboard robot before motor activation.");
}

torch::Tensor StateRL::quatRotateInverse(const torch::Tensor& q, const torch::Tensor& v, const std::string& framework)
{
    torch::Tensor q_w;
    torch::Tensor q_vec;
    if (framework == "isaacsim")
    {
        q_w = q.index({torch::indexing::Slice(), 0});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(1, 4)});
    }
    else if (framework == "isaacgym")
    {
        q_w = q.index({torch::indexing::Slice(), 3});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});
    }
    const c10::IntArrayRef shape = q.sizes();

    const torch::Tensor a = v * (2.0 * torch::pow(q_w, 2) - 1.0).unsqueeze(-1);
    const torch::Tensor b = cross(q_vec, v, -1) * q_w.unsqueeze(-1) * 2.0;
    const torch::Tensor c = q_vec * bmm(q_vec.view({shape[0], 1, 3}), v.view({shape[0], 3, 1})).squeeze(-1) * 2.0;
    return a - b + c;
}

torch::Tensor StateRL::forward()
{
    torch::autograd::GradMode::set_enabled(false);
    
    torch::Tensor final_obs = computeObservation(); // standard production shape: [1, 570]
    if (params_.policy2_fixed_motion_enabled)
    {
        const std::array<float, 8> phase_features = policy2_fixed_motion_adapter_.features();
        torch::Tensor phase_tensor = torch::from_blob(
            const_cast<float*>(phase_features.data()), {1, 8}, torch::kFloat32).clone();
        final_obs = torch::cat({final_obs, phase_tensor}, 1);
        if (final_obs.size(1) != 578)
        {
            throw std::runtime_error("fixed-motion policy2 requires exactly 570+8=578 inputs");
        }
    }
    
    // ==========================================
    // 🌟 自适应推断逻辑
    // ==========================================
    torch::Tensor actions;
    
    if (params_.teacher_virtual_terrain_enabled)
    {
        if (!teacher_reference_loaded_ || final_obs.size(1) != 570)
        {
            throw std::runtime_error("Teacher virtual terrain requires loaded reference and exact 570 history");
        }
        const highstep_virtual_terrain::State& adapter_state = teacher_virtual_adapter_.state();
        const double lookup_progress = teacher_positive_command_seen_
            ? adapter_state.longitudinal_progress : 0.0;
        torch::Tensor virtual_pose = torch::tensor({{
            lookup_progress,
            adapter_state.lateral_offset,
            adapter_state.relative_yaw,
            adapter_state.body_height_offset}}, torch::kFloat32);
        torch::Tensor query = torch::cat({virtual_pose, obs_.dof_pos.to(torch::kFloat32)}, 1);
        torch::Tensor previous_phase = torch::tensor({teacher_virtual_phase_}, torch::kFloat32);
        torch::Tensor previous_progress = torch::tensor({teacher_previous_lookup_progress_}, torch::kFloat32);
        torch::Tensor reference = teacher_virtual_reference_model_.forward(
            {query, previous_phase, teacher_fake_scan_, previous_progress}).toTensor();
        RequireTensorSize(reference, 110, "virtual terrain reference output");
        const torch::Tensor next_scan = reference.index({torch::indexing::Slice(), torch::indexing::Slice(0, 102)});
        const double scan_jump = torch::max(torch::abs(next_scan - teacher_fake_scan_)).item<double>();
        const double next_front_rear_delta = reference[0][102].item<double>();
        const double next_phase = reference[0][103].item<double>();
        const double expected_height = reference[0][104].item<double>();
        const double height_lower = reference[0][105].item<double>();
        const double height_upper = reference[0][106].item<double>();
        const double expected_yaw = reference[0][107].item<double>();
        const double yaw_lower = reference[0][108].item<double>();
        const double yaw_upper = reference[0][109].item<double>();
        if (!std::isfinite(next_front_rear_delta) || !std::isfinite(next_phase)
            || !std::isfinite(expected_height) || !std::isfinite(height_lower)
            || !std::isfinite(height_upper) || !std::isfinite(expected_yaw)
            || !std::isfinite(yaw_lower) || !std::isfinite(yaw_upper)
            || !torch::isfinite(next_scan).all().item<bool>())
        {
            deployment_fault_requires_passive_.store(true, std::memory_order_release);
            throw std::runtime_error("Teacher virtual-terrain reference generated non-finite data");
        }
        if (!teacher_positive_command_seen_ && std::abs(next_phase) > 1.0e-12)
        {
            deployment_fault_requires_passive_.store(false, std::memory_order_release);
            throw std::runtime_error("Teacher virtual-terrain phase advanced before first positive joystick input");
        }
        if (inference_commit_count_.load(std::memory_order_acquire) > 0 && scan_jump > 0.20)
        {
            deployment_fault_requires_passive_.store(
                teacher_virtual_adapter_.contactCriticalPhase() || next_phase >= 0.15,
                std::memory_order_release);
            throw std::runtime_error("virtual terrain scan changed discontinuously");
        }
        teacher_fake_scan_ = next_scan.clone();
        teacher_previous_scan_max_delta_ = scan_jump;
        teacher_front_rear_delta_ = next_front_rear_delta;
        teacher_virtual_phase_ = next_phase;
        teacher_previous_lookup_progress_ = teacher_positive_command_seen_
            ? adapter_state.longitudinal_progress : 0.0;
        const bool phase_envelope_accepted = teacher_virtual_adapter_.acceptReferenceEnvelope(
            teacher_virtual_phase_, expected_height, height_lower, height_upper,
            expected_yaw, yaw_lower, yaw_upper);
        const bool confidence_below_threshold =
            adapter_state.confidence < params_.teacher_adapter_min_confidence;
        const bool confidence_low_too_long = teacher_virtual_adapter_.confidenceLowTooLong();
        teacher_soft_gate_event_mask_ = 0;
        if (!phase_envelope_accepted) teacher_soft_gate_event_mask_ |= 1U;
        if (!teacher_virtual_adapter_.heightInPhaseEnvelope()) teacher_soft_gate_event_mask_ |= 2U;
        if (!teacher_virtual_adapter_.yawInPhaseEnvelope()) teacher_soft_gate_event_mask_ |= 4U;
        if (confidence_below_threshold) teacher_soft_gate_event_mask_ |= 8U;
        if (confidence_low_too_long) teacher_soft_gate_event_mask_ |= 16U;
        if (!teacher_virtual_adapter_.baseDomainValid()) teacher_soft_gate_event_mask_ |= 32U;
        if (teacher_soft_gate_event_mask_ != 0U)
        {
            if (!params_.diagnostic_soft_gates_log_only || !diagnostic_mujoco_backend_verified_)
            {
                deployment_fault_requires_passive_.store(
                    teacher_virtual_adapter_.contactCriticalPhase() || next_phase >= 0.15,
                    std::memory_order_release);
                throw std::runtime_error("virtual terrain adapter confidence/phase safety check failed");
            }
            ++teacher_soft_gate_bypass_count_;
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 250,
                "MUJOCO diagnostic soft gate bypass: mask=%u phase=%.6f confidence=%.6f count=%llu",
                teacher_soft_gate_event_mask_, teacher_virtual_phase_, adapter_state.confidence,
                static_cast<unsigned long long>(teacher_soft_gate_bypass_count_));
        }
        const torch::Tensor gravity = quatRotateInverse(obs_.base_quat, obs_.gravity_vec, params_.framework);
        const torch::Tensor privileged_proprio = torch::cat({
            obs_.ang_vel, gravity, obs_.commands,
            (obs_.dof_pos - params_.default_dof_pos) * params_.dof_pos_scale,
            obs_.dof_vel, obs_.actions}, 1).to(torch::kFloat32);
        RequireTensorSize(privileged_proprio, 57, "Teacher privileged proprioception");
        const torch::Tensor teacher_output = model_.forward(
            {final_obs, privileged_proprio, teacher_fake_scan_}).toTensor();
        RequireTensorSize(teacher_output, 80, "Teacher core output (16 action + 64 latent)");
        actions = teacher_output.index({torch::indexing::Slice(), torch::indexing::Slice(0, 16)});
    }
    else if (params_.policy2_fixed_motion_enabled)
    {
        // The fixed-motion model has one exact deployable input contract.  A
        // fallback would silently run the wrong policy shape on hardware.
        actions = model_.forward({final_obs}).toTensor();
    }
    else try {
        actions = model_.forward({final_obs}).toTensor();
    } catch (const c10::Error&) {
        torch::Tensor dummy_latent = torch::zeros({1, 64}, torch::TensorOptions().dtype(torch::kFloat32).device(final_obs.device()));
        torch::Tensor fallback_input = torch::cat({final_obs, dummy_latent}, 1);
        
        actions = model_.forward({fallback_input}).toTensor();
    }

    // ==========================================
    // 【新增 Debug 发布】：将 default_dof_pos 的偏移量发布到 ROS2
    // ==========================================
    if (obs_dimension_debug_pub_ != nullptr) {
        std_msgs::msg::Float32MultiArray debug_msg;
        
        float current_pos = obs_.dof_pos[0][0].item<float>();
        float wrong_default = init_pos_[0]; // 如果没写 YAML，这就是网络看到的基准
        float correct_default = params_.default_dof_pos[0][0].item<float>(); // 实际使用的基准
        
        debug_msg.data.push_back(current_pos);                                      // [0] 当前真实位置
        debug_msg.data.push_back((current_pos - wrong_default) * params_.dof_pos_scale);   // [1] 如果用 init_pos_，网络看到的相对位置 (几乎为0)
        debug_msg.data.push_back((current_pos - correct_default) * params_.dof_pos_scale); // [2] 实际喂给网络的相对位置 (应该反映真实的物理偏差)
        debug_msg.data.push_back(actions[0][0].item<float>());                      // [3] 动作输出
        
        obs_dimension_debug_pub_->publish(debug_msg);
    }

     // ==========================================
    // 【新增 Debug 发布】：证明 Estimator Scale 错位导致了 20 倍的数值鸿沟
    // ==========================================
    if (scale_mismatch_debug_pub_ != nullptr) {
        std_msgs::msg::Float32MultiArray debug_msg;
        
        // 提取第一个关节(FL_hip)的原始物理速度 (rad/s)
        float raw_vel = obs_.dof_vel[0][0].item<float>();
        
        // 1. C++ 实际喂给网络的缩放后速度 (乘以了 0.05)
        float scaled_vel_fed_to_network = raw_vel * params_.dof_vel_scale; 
        
        // 2. 未修复 Python 代码前，Estimator 内部权重期望看到的速度 (乘以了 1.0)
        float wrong_expected_vel_by_old_estimator = raw_vel * 1.0;         
        
        debug_msg.data.push_back(raw_vel);                             // [0] 真实物理速度
        debug_msg.data.push_back(scaled_vel_fed_to_network);           // [1] 喂给网络的极小值
        debug_msg.data.push_back(wrong_expected_vel_by_old_estimator); // [2] 旧 Estimator 渴望得到的巨大值
        
        scale_mismatch_debug_pub_->publish(debug_msg);
    }

    if (params_.clip_actions_upper.numel() != 0 && params_.clip_actions_lower.numel() != 0)
    {
        return clamp(actions, params_.clip_actions_lower, params_.clip_actions_upper);
    }
    return actions;
}

void StateRL::getState()
{
    RobotState<double> next_state;
    Control next_control;
    if (params_.framework == "isaacgym")
    {
        next_state.imu.quaternion[3] = ctrl_interfaces_.imu_state_interface_[0].get().get_value();
        next_state.imu.quaternion[0] = ctrl_interfaces_.imu_state_interface_[1].get().get_value();
        next_state.imu.quaternion[1] = ctrl_interfaces_.imu_state_interface_[2].get().get_value();
        next_state.imu.quaternion[2] = ctrl_interfaces_.imu_state_interface_[3].get().get_value();
    }
    else if (params_.framework == "isaacsim")
    {
        next_state.imu.quaternion[0] = ctrl_interfaces_.imu_state_interface_[0].get().get_value();
        next_state.imu.quaternion[1] = ctrl_interfaces_.imu_state_interface_[1].get().get_value();
        next_state.imu.quaternion[2] = ctrl_interfaces_.imu_state_interface_[2].get().get_value();
        next_state.imu.quaternion[3] = ctrl_interfaces_.imu_state_interface_[3].get().get_value();
    }

    next_state.imu.gyroscope[0] = ctrl_interfaces_.imu_state_interface_[4].get().get_value();
    next_state.imu.gyroscope[1] = ctrl_interfaces_.imu_state_interface_[5].get().get_value();
    next_state.imu.gyroscope[2] = ctrl_interfaces_.imu_state_interface_[6].get().get_value();

    next_state.imu.accelerometer[0] = ctrl_interfaces_.imu_state_interface_[7].get().get_value();
    next_state.imu.accelerometer[1] = ctrl_interfaces_.imu_state_interface_[8].get().get_value();
    next_state.imu.accelerometer[2] = ctrl_interfaces_.imu_state_interface_[9].get().get_value();

    for (int i = 0; i < 16; i++)
    {
        next_state.motor_state.q[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        next_state.motor_state.dq[i] = ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value();
        next_state.motor_state.tauEst[i] = ctrl_interfaces_.joint_effort_state_interface_[i].get().get_value();
    }

    next_control.x = ctrl_interfaces_.control_inputs_.lx;
    next_control.y = ctrl_interfaces_.control_inputs_.ly;
    next_control.yaw = -ctrl_interfaces_.control_inputs_.rx;

    std::lock_guard<std::mutex> lock(state_mutex_);
    robot_state_ = std::move(next_state);
    control_ = next_control;
}

void StateRL::runModel(const std::uint64_t command_epoch)
{
    if (!running_.load(std::memory_order_acquire)
        || command_epoch != command_epoch_.load(std::memory_order_acquire))
    {
        return;
    }

    std::lock_guard<std::mutex> inference_lock(inference_mutex_);
    if (!running_.load(std::memory_order_acquire)
        || command_epoch != command_epoch_.load(std::memory_order_acquire))
    {
        return;
    }

    RobotState<double> state_snapshot;
    Control control_snapshot;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_snapshot = robot_state_;
        control_snapshot = control_;
    }

    if (enable_estimator_)
    {
        obs_.lin_vel = torch::from_blob(estimator_->getVelocity().data(), {3}, torch::kDouble).clone().
            to(torch::kFloat).unsqueeze(0);
    }
    obs_.ang_vel = torch::tensor(state_snapshot.imu.gyroscope).unsqueeze(0);
    double model_command_x = control_snapshot.x;
    if (params_.policy2_fixed_motion_enabled)
    {
        policy2_command_sample_ = policy2_fixed_motion_adapter_.resolveCommand(control_snapshot.x);
        model_command_x = policy2_command_sample_.model_vx;
    }
    else if (params_.teacher_virtual_terrain_enabled)
    {
        // Continuous calibration only: no latch, playback, or fixed command.
        // Release remains zero and partial stick remains proportional.
        model_command_x = control_snapshot.x * params_.teacher_policy2_command_x_gain;
        if (control_snapshot.x > 0.05)
        {
            teacher_positive_command_seen_ = true;
        }
        // The accepted single-action contract ends at phase 1 and waits for
        // the operator to switch to Fixed Stand.  Keep recording raw joystick
        // input, but never feed continued full-stick vx into the terminal hold.
        if (teacher_virtual_phase_ >= 1.0 - 1.0e-6)
        {
            model_command_x = 0.0;
        }
        std::array<double, 4> quaternion{};
        std::array<double, 3> acceleration{};
        std::array<double, 16> joint_position{};
        std::array<double, 16> joint_velocity{};
        std::copy_n(state_snapshot.imu.quaternion.begin(), 4, quaternion.begin());
        std::copy_n(state_snapshot.imu.accelerometer.begin(), 3, acceleration.begin());
        std::copy_n(state_snapshot.motor_state.q.begin(), 16, joint_position.begin());
        std::copy_n(state_snapshot.motor_state.dq.begin(), 16, joint_velocity.begin());
        const double measured_vx = enable_estimator_ ? obs_.lin_vel[0][0].item<double>() : 0.0;
        teacher_virtual_adapter_.update(
            model_command_x,
            control_snapshot.y,
            control_snapshot.yaw,
            quaternion,
            acceleration,
            joint_position,
            joint_velocity,
            measured_vx,
            enable_estimator_,
            static_cast<double>(params_.decimation) / ctrl_interfaces_.frequency_);
        const double w = quaternion[0], x = quaternion[1], y = quaternion[2], z = quaternion[3];
        const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
        const double angular_rate = std::sqrt(
            state_snapshot.imu.gyroscope[0] * state_snapshot.imu.gyroscope[0]
            + state_snapshot.imu.gyroscope[1] * state_snapshot.imu.gyroscope[1]
            + state_snapshot.imu.gyroscope[2] * state_snapshot.imu.gyroscope[2]);
        if (std::abs(roll) > 0.70 || std::abs(pitch) > 0.85 || angular_rate > 8.0)
        {
            deployment_fault_requires_passive_.store(true, std::memory_order_release);
            throw std::runtime_error("Teacher adapter detected rapid tilt; passive exit required");
        }
    }
    obs_.commands = torch::tensor({{model_command_x, control_snapshot.y, control_snapshot.yaw}});
    obs_.base_quat = torch::tensor(state_snapshot.imu.quaternion).unsqueeze(0);
    obs_.dof_pos = torch::tensor(state_snapshot.motor_state.q).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);
    obs_.dof_vel = torch::tensor(state_snapshot.motor_state.dq).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);

    const torch::Tensor clamped_actions = forward();
    RequireTensorSize(clamped_actions, params_.num_of_dofs, "policy output");
    const torch::Tensor policy_raw_actions = clamped_actions.clone();

    for (const int i : params_.hip_scale_reduction_indices)
    {
        clamped_actions[0][i] *= params_.hip_scale_reduction;
    }

    obs_.actions = clamped_actions;

    const torch::Tensor actions_scaled = clamped_actions * params_.action_scale;
    // torch::Tensor output_torques = params_.rl_kp * (actions_scaled + params_.default_dof_pos - obs_.dof_pos) - params_.rl_kd * obs_.dof_vel;
    // output_torques = clamp(output_torques, -(params_.torque_limits), params_.torque_limits);

    output_dof_pos_ = actions_scaled + params_.default_dof_pos;
    if (params_.teacher_virtual_terrain_enabled)
    {
        // Exact production phased prior, applied once and only once after the
        // Teacher pre-prior action is mapped to physical joint targets.
        const torch::Tensor scan_grid = teacher_fake_scan_.reshape({1, 6, 17});
        const torch::Tensor front_mean = scan_grid.index({0, torch::indexing::Slice(), torch::indexing::Slice(11, 17)}).mean();
        const torch::Tensor rear_mean = scan_grid.index({0, torch::indexing::Slice(), torch::indexing::Slice(0, 7)}).mean();
        auto smoothstep = [](const double value)
        {
            const double x = std::clamp(value, 0.0, 1.0);
            return x * x * (3.0 - 2.0 * x);
        };
        const double height_gate = smoothstep((rear_mean.item<double>() - front_mean.item<double>() - 0.06) / 0.14);
        const double command_gate = std::clamp((model_command_x - 0.06) / 0.22, 0.0, 1.0);
        const double commit_gate = smoothstep((teacher_front_rear_delta_ - 0.04) / 0.14);
        const double reach_gate = height_gate * command_gate * (1.0 - commit_gate);
        const double push_gate = height_gate * command_gate * commit_gate;
        constexpr std::array<double, 4> reach_bias = {-0.020, -0.020, 0.002, 0.002};
        constexpr std::array<double, 4> push_bias = {0.002, 0.002, -0.022, -0.022};
        for (std::size_t box = 0; box < 4; ++box)
        {
            const std::int64_t index = static_cast<std::int64_t>(12 + box);
            const double biased = output_dof_pos_[0][index].item<double>()
                + reach_gate * reach_bias[box] + push_gate * push_bias[box];
            output_dof_pos_[0][index] = std::clamp(biased, 0.0, 0.060);
        }
    }
    else if (params_.policy2_fixed_motion_enabled)
    {
        const std::array<float, 4> box_bias = policy2_fixed_motion_adapter_.boxBiasMeters();
        for (std::size_t box = 0; box < box_bias.size(); ++box)
        {
            output_dof_pos_[0][static_cast<std::int64_t>(12 + box)] += box_bias[box];
        }
    }
    if (!torch::isfinite(output_dof_pos_).all().item<bool>())
    {
        deployment_fault_requires_passive_.store(
            params_.teacher_virtual_terrain_enabled && teacher_virtual_adapter_.contactCriticalPhase(),
            std::memory_order_release);
        throw std::runtime_error("Policy generated a non-finite joint target");
    }
    const bool rear_hip_guard_state_enabled =
        !params_.rear_hip_guard_policy2_only || state_name == FSMStateName::RLPOLICY2;
    if (params_.rear_hip_guard_enabled && rear_hip_guard_state_enabled
        && params_.num_of_dofs >= 4 && control_snapshot.x > params_.rear_hip_guard_min_cmd_x)
    {
        const float original_rl = output_dof_pos_[0][2].item<float>();
        const float original_rr = output_dof_pos_[0][3].item<float>();
        const float min_abs = static_cast<float>(std::max(0.0, params_.rear_hip_guard_min_abs));
        const float blend = static_cast<float>(std::min(1.0, std::max(0.0, params_.rear_hip_guard_blend)));
        const float guarded_rl = std::max(original_rl, min_abs);
        const float guarded_rr = std::min(original_rr, -min_abs);
        output_dof_pos_[0][2] = original_rl * (1.0f - blend) + guarded_rl * blend;
        output_dof_pos_[0][3] = original_rr * (1.0f - blend) + guarded_rr * blend;

        const float max_guard_delta = std::max(
            std::abs(output_dof_pos_[0][2].item<float>() - original_rl),
            std::abs(output_dof_pos_[0][3].item<float>() - original_rr));
        if (max_guard_delta > 1.0e-5f
            && rear_hip_guard_warning_counter_++ % std::max(1, params_.rear_hip_guard_warning_interval) == 0)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("StateRL"),
                "Rear hip guard adjusted highstep targets. RL %.4f->%.4f, RR %.4f->%.4f",
                original_rl,
                output_dof_pos_[0][2].item<float>(),
                original_rr,
                output_dof_pos_[0][3].item<float>());
        }

        if (rear_hip_guard_debug_pub_ != nullptr) {
            std_msgs::msg::Float32MultiArray debug_msg;
            debug_msg.data.push_back(original_rl);
            debug_msg.data.push_back(output_dof_pos_[0][2].item<float>());
            debug_msg.data.push_back(original_rr);
            debug_msg.data.push_back(output_dof_pos_[0][3].item<float>());
            debug_msg.data.push_back(max_guard_delta);
            debug_msg.data.push_back(static_cast<float>(control_snapshot.x));
            rear_hip_guard_debug_pub_->publish(debug_msg);
        }
    }

    const torch::Tensor unclamped_output_dof_pos = output_dof_pos_.clone();
    torch::Tensor effective_lower = params_.physical_dof_pos_lower;
    torch::Tensor effective_upper = params_.physical_dof_pos_upper;
    if (params_.output_dof_pos_lower.numel() != 0)
    {
        effective_lower = torch::maximum(effective_lower, params_.output_dof_pos_lower);
        effective_upper = torch::minimum(effective_upper, params_.output_dof_pos_upper);
    }
    output_dof_pos_ = clamp(output_dof_pos_, effective_lower, effective_upper);
    const float clamp_delta = torch::max(torch::abs(output_dof_pos_ - unclamped_output_dof_pos)).item<float>();

    // Compare only commands that can actually reach the actuator.  Comparing
    // an unclamped policy excursion with the previous clamped command creates
    // false passive exits even when the transmitted target is unchanged.
    if (params_.teacher_virtual_terrain_enabled
        && inference_commit_count_.load(std::memory_order_acquire) > 0)
    {
        int violating_joint = -1;
        double violating_old_clamped = 0.0;
        double violating_new_clamped = 0.0;
        double violating_unclamped = 0.0;
        double violating_threshold = 0.0;
        {
            std::lock_guard<std::mutex> command_lock(command_mutex_);
            for (int i = 0; i < params_.num_of_dofs; ++i)
            {
                const double old_clamped = robot_command_.motor_command.q[i];
                const double new_clamped = output_dof_pos_[0][i].item<double>();
                const double threshold = i < 12 ? 0.35 : 0.025;
                if (std::abs(new_clamped - old_clamped) > threshold)
                {
                    violating_joint = i;
                    violating_old_clamped = old_clamped;
                    violating_new_clamped = new_clamped;
                    violating_unclamped = unclamped_output_dof_pos[0][i].item<double>();
                    violating_threshold = threshold;
                    break;
                }
            }
        }
        if (violating_joint >= 0)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Teacher virtual-terrain clamped target jump: joint=%s old_clamped=%.9f "
                "new_clamped=%.9f unclamped=%.9f phase=%.9f threshold=%.9f",
                verified_joint_names_[static_cast<std::size_t>(violating_joint)].c_str(),
                violating_old_clamped, violating_new_clamped, violating_unclamped,
                teacher_virtual_phase_, violating_threshold);
            deployment_fault_requires_passive_.store(
                teacher_virtual_adapter_.contactCriticalPhase(), std::memory_order_release);
            throw std::runtime_error("Teacher virtual-terrain target jump exceeded safety envelope");
        }
    }

    RobotCommand<double> next_command;
    for (int i = 0; i < params_.num_of_dofs; ++i)
    {
        next_command.motor_command.q[i] = output_dof_pos_[0][i].item<double>();
        next_command.motor_command.dq[i] = 0.0;
        next_command.motor_command.kp[i] = params_.rl_kp[0][i].item<double>();
        next_command.motor_command.kd[i] = params_.rl_kd[0][i].item<double>();
        next_command.motor_command.tau[i] = 0.0;
    }

    std::uint64_t sequence = 0;
    std::uint64_t inference_sequence = 0;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (!running_.load(std::memory_order_acquire)
            || command_epoch != command_epoch_.load(std::memory_order_acquire))
        {
            return;
        }
        robot_command_ = std::move(next_command);
        command_valid_.store(true, std::memory_order_release);
        inference_command_valid_.store(true, std::memory_order_release);
        sequence = command_sequence_.fetch_add(1, std::memory_order_acq_rel) + 1;
        inference_sequence = inference_commit_count_.fetch_add(1, std::memory_order_acq_rel) + 1;
    }

    double expected_latency = -1.0;
    const double latency_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - enter_steady_time_).count();
    if (first_inference_latency_ms_.compare_exchange_strong(
            expected_latency, latency_ms, std::memory_order_acq_rel))
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "First inference command committed atomically for %s epoch=%llu sequence=%llu latency_ms=%.3f",
            state_name_string.c_str(), static_cast<unsigned long long>(command_epoch),
            static_cast<unsigned long long>(sequence), latency_ms);
    }

    last_clamp_max_delta_.store(clamp_delta, std::memory_order_release);
    if ((params_.policy2_fixed_motion_enabled || params_.teacher_virtual_terrain_enabled
         || params_.policy2_contract_trace_enabled)
        && policy2_contract_trace_pub_)
    {
        // Layout: schema, command sequence, inference sequence, one-shot step,
        // raw joystick vx (record-only), actual model vx, phase, one-shot enabled, then
        // policy raw[16], post-prior/pre-physical-clamp target[16], and final
        // physical-clamp target[16].  This is published for every inference.
        std_msgs::msg::Float32MultiArray trace;
        trace.data.reserve(8 + 3 * params_.num_of_dofs);
        trace.data.push_back(1.0F);
        trace.data.push_back(static_cast<float>(sequence));
        trace.data.push_back(static_cast<float>(inference_sequence));
        trace.data.push_back(params_.policy2_fixed_motion_enabled
            ? static_cast<float>(policy2_command_sample_.diagnostic_step) : -1.0F);
        trace.data.push_back(static_cast<float>(control_snapshot.x));
        trace.data.push_back(static_cast<float>(model_command_x));
        trace.data.push_back(params_.teacher_virtual_terrain_enabled
            ? static_cast<float>(teacher_virtual_phase_)
            : (params_.policy2_fixed_motion_enabled
                ? static_cast<float>(policy2_fixed_motion_adapter_.phase()) : -1.0F));
        trace.data.push_back(params_.policy2_one_shot_enabled ? 1.0F : 0.0F);
        for (int i = 0; i < params_.num_of_dofs; ++i)
        {
            trace.data.push_back(policy_raw_actions[0][i].item<float>());
        }
        for (int i = 0; i < params_.num_of_dofs; ++i)
        {
            trace.data.push_back(unclamped_output_dof_pos[0][i].item<float>());
        }
        for (int i = 0; i < params_.num_of_dofs; ++i)
        {
            trace.data.push_back(output_dof_pos_[0][i].item<float>());
        }
        policy2_contract_trace_pub_->publish(trace);
    }
    if (params_.teacher_virtual_terrain_enabled && teacher_virtual_adapter_trace_pub_)
    {
        const auto& state = teacher_virtual_adapter_.state();
        std_msgs::msg::Float32MultiArray trace;
        trace.data.reserve(123);
        trace.data = {
            1.0F,
            static_cast<float>(sequence),
            static_cast<float>(state.longitudinal_progress),
            static_cast<float>(state.lateral_offset),
            static_cast<float>(state.relative_yaw),
            static_cast<float>(state.body_height_offset),
            static_cast<float>(state.confidence),
            static_cast<float>(teacher_virtual_phase_),
            static_cast<float>(teacher_front_rear_delta_),
            static_cast<float>(teacher_previous_scan_max_delta_),
        };
        const torch::Tensor scan_cpu = teacher_fake_scan_.to(torch::kCPU).contiguous();
        for (int i = 0; i < 102; ++i)
        {
            trace.data.push_back(scan_cpu[0][i].item<float>());
        }
        trace.data.push_back(static_cast<float>(teacher_virtual_adapter_.expectedHeight()));
        trace.data.push_back(static_cast<float>(teacher_virtual_adapter_.heightResidual()));
        trace.data.push_back(static_cast<float>(teacher_virtual_adapter_.expectedYaw()));
        trace.data.push_back(static_cast<float>(teacher_virtual_adapter_.yawResidual()));
        trace.data.push_back(params_.diagnostic_soft_gates_log_only ? 1.0F : 0.0F);
        trace.data.push_back(diagnostic_mujoco_backend_verified_ ? 1.0F : 0.0F);
        trace.data.push_back(static_cast<float>(teacher_soft_gate_event_mask_));
        trace.data.push_back(static_cast<float>(teacher_soft_gate_bypass_count_));
        trace.data.push_back(teacher_virtual_adapter_.baseDomainValid() ? 1.0F : 0.0F);
        trace.data.push_back(teacher_virtual_adapter_.heightInPhaseEnvelope() ? 1.0F : 0.0F);
        trace.data.push_back(teacher_virtual_adapter_.yawInPhaseEnvelope() ? 1.0F : 0.0F);
        trace.data.push_back(teacher_positive_command_seen_ ? 1.0F : 0.0F);
        teacher_virtual_adapter_trace_pub_->publish(trace);
    }
    if (clamp_delta > 1.0e-5F)
    {
        const std::uint64_t clamp_count = clamp_event_count_.fetch_add(1, std::memory_order_acq_rel) + 1;
        float observed_max = max_clamp_delta_.load(std::memory_order_acquire);
        while (clamp_delta > observed_max
               && !max_clamp_delta_.compare_exchange_weak(
                   observed_max, clamp_delta, std::memory_order_acq_rel))
        {
        }

        if (output_clamp_warning_counter_++ % 200 == 0)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "RL target clamp active for %s. event=%llu max_delta=%.5f; policy must be refined under the same limits.",
                state_name_string.c_str(), static_cast<unsigned long long>(clamp_count), clamp_delta);
        }

        if (target_clamp_debug_pub_)
        {
            // Layout: schema, state enum, command sequence, inference sequence,
            // clamp event count, max delta, then raw[16], clamped[16]. Every
            // inference has a sequence, so a bag can recover clamp fraction and
            // consecutive-event duration without assuming a fixed loop rate.
            std_msgs::msg::Float32MultiArray debug_msg;
            debug_msg.data.reserve(6 + 2 * params_.num_of_dofs);
            debug_msg.data.push_back(1.0F);
            debug_msg.data.push_back(static_cast<float>(static_cast<int>(state_name)));
            debug_msg.data.push_back(static_cast<float>(sequence));
            debug_msg.data.push_back(static_cast<float>(inference_sequence));
            debug_msg.data.push_back(static_cast<float>(clamp_count));
            debug_msg.data.push_back(clamp_delta);
            for (int i = 0; i < params_.num_of_dofs; ++i)
            {
                debug_msg.data.push_back(unclamped_output_dof_pos[0][i].item<float>());
            }
            for (int i = 0; i < params_.num_of_dofs; ++i)
            {
                debug_msg.data.push_back(output_dof_pos_[0][i].item<float>());
            }
            target_clamp_debug_pub_->publish(debug_msg);
        }
    }
    publishCommandSafetyDebug(true);
}

bool StateRL::validatePolicy2TriggerState(const RobotState<double>& state_snapshot) const
{
    if (state_name != FSMStateName::RLPOLICY2
        || state_snapshot.motor_state.q.size() < 16
        || state_snapshot.motor_state.dq.size() < 16)
    {
        return false;
    }
    for (int i = 0; i < 16; ++i)
    {
        const double q = state_snapshot.motor_state.q[static_cast<std::size_t>(i)];
        const double dq = state_snapshot.motor_state.dq[static_cast<std::size_t>(i)];
        const bool box = i >= 12;
        const double q_tolerance = box
            ? params_.policy2_trigger_box_tolerance
            : params_.policy2_trigger_revolute_tolerance;
        const double dq_limit = box
            ? params_.policy2_trigger_box_velocity_limit
            : params_.policy2_trigger_revolute_velocity_limit;
        if (!std::isfinite(q) || !std::isfinite(dq)
            || std::abs(q - init_pos_[i]) > q_tolerance
            || std::abs(dq) > dq_limit)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Policy2 trigger state rejected at %s: q=%.8f stand=%.8f tolerance=%.8f dq=%.8f limit=%.8f",
                verified_joint_names_[static_cast<std::size_t>(i)].c_str(), q, init_pos_[i],
                q_tolerance, dq, dq_limit);
            return false;
        }
    }
    return true;
}

bool StateRL::initializeHoldCommand(const RobotState<double>& state_snapshot)
{
    RobotCommand<double> hold_command;
    for (int i = 0; i < 16; i++)
    {
        const double position = state_snapshot.motor_state.q[i];
        double lower = params_.physical_dof_pos_lower[0][i].item<double>();
        double upper = params_.physical_dof_pos_upper[0][i].item<double>();
        if (params_.output_dof_pos_lower.numel() != 0)
        {
            lower = std::max(lower, params_.output_dof_pos_lower[0][i].item<double>());
            upper = std::min(upper, params_.output_dof_pos_upper[0][i].item<double>());
        }
        if (!std::isfinite(position) || position < lower || position > upper)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Cannot form RL hold: %s measured q=%.8f is not within configured physical target [%.8f, %.8f]",
                verified_joint_names_[static_cast<std::size_t>(i)].c_str(), position, lower, upper);
            return false;
        }
        hold_command.motor_command.q[i] = position;
        hold_command.motor_command.dq[i] = 0.0;
        hold_command.motor_command.kp[i] = params_.rl_kp[0][i].item<double>();
        hold_command.motor_command.kd[i] = params_.rl_kd[0][i].item<double>();
        hold_command.motor_command.tau[i] = 0.0;
    }

    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        robot_command_ = std::move(hold_command);
        command_valid_.store(true, std::memory_order_release);
        inference_command_valid_.store(false, std::memory_order_release);
        command_sequence_.fetch_add(1, std::memory_order_acq_rel);
    }
    RCLCPP_INFO(
        node_->get_logger(),
        "Current-position hold committed for %s with configured non-zero RL gains; waiting for first inference",
        state_name_string.c_str());
    return true;
}

void StateRL::setCommand()
{
    RobotCommand<double> command_snapshot;
    // Keep exit(), model commit and this validate+write transaction mutually
    // exclusive. This prevents lifecycle deactivation from racing a validated
    // snapshot and allowing an old epoch to write afterward.
    std::unique_lock<std::mutex> command_lock(command_mutex_);
    if (!command_valid_.load(std::memory_order_acquire))
    {
        command_lock.unlock();
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "RL command is invalid; preserving the previous hardware-interface command and requesting passive");
        deployment_fault_.store(true, std::memory_order_release);
        publishCommandSafetyDebug(true);
        return;
    }
    command_snapshot = robot_command_;

    // Validate the complete 16-joint frame before writing any interface. This
    // prevents a bad later joint from leaving a mixed old/new half-frame.
    for (int i = 0; i < 16; ++i)
    {
        const double q = command_snapshot.motor_command.q[i];
        const double dq = command_snapshot.motor_command.dq[i];
        const double kp = command_snapshot.motor_command.kp[i];
        const double kd = command_snapshot.motor_command.kd[i];
        const double tau = command_snapshot.motor_command.tau[i];
        double lower = params_.physical_dof_pos_lower[0][i].item<double>();
        double upper = params_.physical_dof_pos_upper[0][i].item<double>();
        if (params_.output_dof_pos_lower.numel() != 0)
        {
            lower = std::max(lower, params_.output_dof_pos_lower[0][i].item<double>());
            upper = std::min(upper, params_.output_dof_pos_upper[0][i].item<double>());
        }
        const double torque_limit = params_.torque_limits[0][i].item<double>();
        if (!std::isfinite(q) || !std::isfinite(dq) || !std::isfinite(kp)
            || !std::isfinite(kd) || !std::isfinite(tau)
            || q < lower || q > upper || kp <= 0.0 || kd < 0.0
            || std::abs(tau) > torque_limit)
        {
            running_.store(false, std::memory_order_release);
            command_epoch_.fetch_add(1, std::memory_order_acq_rel);
            command_valid_.store(false, std::memory_order_release);
            inference_command_valid_.store(false, std::memory_order_release);
            deployment_fault_.store(true, std::memory_order_release);
            command_lock.unlock();
            RCLCPP_ERROR(
                node_->get_logger(),
                "Invalid complete-frame command for %s: q=%.8f allowed=[%.8f,%.8f] dq=%.8f "
                "kp=%.8f kd=%.8f tau=%.8f torque_limit=%.8f; no joint command was written",
                verified_joint_names_[static_cast<std::size_t>(i)].c_str(), q, lower, upper, dq,
                kp, kd, tau, torque_limit);
            publishCommandSafetyDebug(true);
            return;
        }
    }

    // All 16 q/dq/kp/kd/tau tuples passed; write the frame in a second pass.
    for (int i = 0; i < 16; ++i)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().
                                                                            set_value(
                                                                                command_snapshot.motor_command.q[i]);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(
            command_snapshot.motor_command.dq[i]);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(
            command_snapshot.motor_command.kp[i]);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(
            command_snapshot.motor_command.kd[i]);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().
                                                                          set_value(
                                                                              command_snapshot.motor_command.tau[i]);
    }
    command_lock.unlock();

    ++control_tick_counter_;
    if (control_tick_counter_ % 20 == 0)
    {
        publishCommandSafetyDebug();
    }
    if (control_tick_counter_ % 200 == 0)
    {
        publishDeploymentManifest();
    }
}

std::string StateRL::buildDeploymentManifest() const
{
    std::ostringstream stream;
    stream << std::setprecision(10)
           << "{\"schema_version\":1"
           << ",\"state\":\"" << JsonEscape(state_name_string) << "\""
           << ",\"state_enum\":" << static_cast<int>(state_name)
           << ",\"policy_role\":\""
           << (state_name == FSMStateName::RLPOLICY2 ? "policy2" : "policy1") << "\""
           << ",\"model_name_key\":\"" << JsonEscape(resolved_model_name_key_) << "\""
           << ",\"model_folder_parameter\":\"" << JsonEscape(model_folder_parameter_) << "\""
           << ",\"model_path\":\"" << JsonEscape(resolved_model_path_) << "\""
           << ",\"model_fingerprint\":\"" << JsonEscape(model_fingerprint_) << "\""
           << ",\"config_path\":\"" << JsonEscape(config_file_path_) << "\""
           << ",\"config_fingerprint\":\"" << JsonEscape(config_fingerprint_) << "\""
           << ",\"framework\":\"" << JsonEscape(params_.framework) << "\""
           << ",\"decimation\":" << params_.decimation
           << ",\"num_dofs\":" << params_.num_of_dofs
           << ",\"build_date\":\"" << __DATE__ << " " << __TIME__ << "\""
           << ",\"fingerprint_algorithm\":\"fnv1a64\""
           << ",\"joint_order\":[";
    for (std::size_t i = 0; i < verified_joint_names_.size(); ++i)
    {
        if (i != 0)
        {
            stream << ',';
        }
        stream << '"' << JsonEscape(verified_joint_names_[i]) << '"';
    }
    stream << ']'
           << ",\"rl_kp\":" << TensorToJson(params_.rl_kp)
           << ",\"rl_kd\":" << TensorToJson(params_.rl_kd)
           << ",\"action_scale\":" << TensorToJson(params_.action_scale)
           << ",\"default_dof_pos\":" << TensorToJson(params_.default_dof_pos)
           << ",\"physical_dof_pos_lower\":" << TensorToJson(params_.physical_dof_pos_lower)
           << ",\"physical_dof_pos_upper\":" << TensorToJson(params_.physical_dof_pos_upper)
           << ",\"output_dof_pos_lower\":" << TensorToJson(params_.output_dof_pos_lower)
           << ",\"output_dof_pos_upper\":" << TensorToJson(params_.output_dof_pos_upper)
           << ",\"clip_actions_lower\":" << TensorToJson(params_.clip_actions_lower)
           << ",\"clip_actions_upper\":" << TensorToJson(params_.clip_actions_upper)
           << ",\"rear_hip_guard_enabled\":" << (params_.rear_hip_guard_enabled ? "true" : "false")
           << ",\"policy2_fixed_motion_enabled\":" << (params_.policy2_fixed_motion_enabled ? "true" : "false")
           << ",\"policy2_fixed_motion_input_dim\":" << (params_.policy2_fixed_motion_enabled ? 578 : 570)
           << ",\"policy2_fixed_motion_phase\":" << policy2_fixed_motion_adapter_.phase()
           << ",\"policy2_one_shot_enabled\":"
           << (params_.policy2_one_shot_enabled ? "true" : "false")
           << ",\"policy2_contract_trace_enabled\":"
           << (params_.policy2_contract_trace_enabled ? "true" : "false")
           << ",\"policy2_one_shot_sequence\":\"21x0+59x0.7200000286+58x0\""
           << ",\"teacher_virtual_terrain_enabled\":"
           << (params_.teacher_virtual_terrain_enabled ? "true" : "false")
           << ",\"teacher_virtual_reference_model\":\""
           << JsonEscape(params_.teacher_virtual_reference_model) << "\""
           << ",\"teacher_actor_history_dim\":570"
           << ",\"teacher_privileged_proprio_dim\":57"
           << ",\"teacher_fake_scan_dim\":102"
           << ",\"teacher_prior_application_count\":1"
           << ",\"diagnostic_soft_gates_log_only\":"
           << (params_.diagnostic_soft_gates_log_only ? "true" : "false")
           << ",\"diagnostic_runtime_backend\":\""
           << JsonEscape(params_.diagnostic_runtime_backend) << "\""
           << ",\"diagnostic_mujoco_backend_verified\":"
           << (diagnostic_mujoco_backend_verified_ ? "true" : "false")
           << ",\"teacher_runtime_world_truth_inputs\":[]"
           << '}';
    return stream.str();
}

void StateRL::publishDeploymentManifest()
{
    if (!deployment_manifest_pub_)
    {
        return;
    }
    std_msgs::msg::String message;
    message.data = deployment_manifest_;
    deployment_manifest_pub_->publish(message);
}

void StateRL::publishCommandSafetyDebug(const bool force)
{
    if (!command_safety_debug_pub_)
    {
        return;
    }
    if (!force && control_tick_counter_ % 20 != 0)
    {
        return;
    }
    // Layout: schema, state enum, epoch, running, command_valid,
    // inference_valid, hold_active, fault, command sequence, inference commits, clamp_count,
    // last_clamp_delta, max_clamp_delta, first_inference_latency_ms.
    std_msgs::msg::Float32MultiArray message;
    message.data = {
        1.0F,
        static_cast<float>(static_cast<int>(state_name)),
        static_cast<float>(command_epoch_.load(std::memory_order_acquire)),
        running_.load(std::memory_order_acquire) ? 1.0F : 0.0F,
        command_valid_.load(std::memory_order_acquire) ? 1.0F : 0.0F,
        inference_command_valid_.load(std::memory_order_acquire) ? 1.0F : 0.0F,
        command_valid_.load(std::memory_order_acquire)
                && !inference_command_valid_.load(std::memory_order_acquire) ? 1.0F : 0.0F,
        deployment_fault_.load(std::memory_order_acquire) ? 1.0F : 0.0F,
        static_cast<float>(command_sequence_.load(std::memory_order_acquire)),
        static_cast<float>(inference_commit_count_.load(std::memory_order_acquire)),
        static_cast<float>(clamp_event_count_.load(std::memory_order_acquire)),
        last_clamp_max_delta_.load(std::memory_order_acquire),
        max_clamp_delta_.load(std::memory_order_acquire),
        static_cast<float>(first_inference_latency_ms_.load(std::memory_order_acquire))};
    command_safety_debug_pub_->publish(message);
}
