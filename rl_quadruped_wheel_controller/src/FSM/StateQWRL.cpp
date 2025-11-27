//
// Created by ray on 2025-07-25.
//

#include "rl_quadruped_wheel_controller/FSM/StateQWRL.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>

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
    std::vector<T> values;
    for (const auto& val : node)
    {
        values.push_back(val.as<T>());
    }

    if (framework == "isaacsim")
    {
        std::vector<T> transposed_values(cols * rows);
        for (int r = 0; r < rows; ++r)
        {
            for (int c = 0; c < cols; ++c)
            {
                transposed_values[c * rows + r] = values[r * cols + c];
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

StateQWRL::StateQWRL(CtrlInterfaces& ctrl_interfaces,
                 CtrlComponent& ctrl_component,
                 const std::vector<double>& target_pos) :
    FSMState(FSMStateName::QWRL, "qw rl", ctrl_interfaces),
    node_(ctrl_component.node_),
    enable_estimator_(ctrl_component.enable_estimator_),
    estimator_(ctrl_component.estimator_)
{
    if (!node_->has_parameter("robot_pkg")) {
        node_->declare_parameter("robot_pkg", robot_pkg_);
    }
    if (!node_->has_parameter("model_folder")) {
        node_->declare_parameter("model_folder", model_folder_);
    }
    if (!node_->has_parameter("use_rl_thread")) {
        node_->declare_parameter("use_rl_thread", use_rl_thread_);
    }

    robot_pkg_ = node_->get_parameter("robot_pkg").as_string();
    model_folder_ = node_->get_parameter("model_folder").as_string();
    use_rl_thread_ = node_->get_parameter("use_rl_thread").as_bool();

    RCLCPP_INFO(node_->get_logger(), "Using robot model from %s", robot_pkg_.c_str());
    const std::string package_share_directory = ament_index_cpp::get_package_share_directory(robot_pkg_);
    const std::string model_path = package_share_directory + "/config/" + model_folder_;

    for (int i = 0; i < 20; i++)
    {
        init_pos_[i] = target_pos[i];
    }
    // RCLCPP_ERROR(node_->get_logger(), "Here init pose set!!!!!!!!");
    // read params from yaml
    loadYaml(model_path);

    if (!params_.observations_history.empty())
    {
        history_obs_buf_ = std::make_shared<ObservationBuffer>(1, params_.num_observations,
                                                               params_.observations_history.size());
    }

    RCLCPP_INFO(node_->get_logger(), "Model loading: %s", params_.model_name.c_str());
    model_ = torch::jit::load(model_path + "/" + params_.model_name);
    gru_hidden_state_ = torch::zeros({num_layers_, 1, hidden_size_}, torch::kFloat32);

    // for (const auto &param: model_.parameters()) {
    //     std::cout << "Parameter dtype: " << param.dtype() << std::endl;
    // }


    if (use_rl_thread_)
    {
        rl_thread_ = std::thread([&]{
            while (true)
            {
                try
                {
                    executeAndSleep(
                        [&]
                        {
                            if (running_)
                            {
                                runModel();
                            }
                        },
                        ctrl_interfaces_.frequency_ / params_.decimation);
                }
                catch (const std::exception& e)
                {
                    running_ = false;
                    RCLCPP_ERROR(rclcpp::get_logger("StateRL"), "Error in RL thread: %s", e.what());
                }
            }
        });
        setThreadPriority(60, rl_thread_);
    }
}

void StateQWRL::enter()
{
    // Init observations
    obs_.lin_vel = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.ang_vel = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    obs_.commands = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.pose_commands = torch::tensor({{0.55, 0.0, 0.35, 1.0, 0.0, 0.0, 0.0}});
    obs_.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}});
    obs_.dof_pos = params_.default_dof_pos;
    obs_.dof_pos_leg = params_.default_dof_pos_leg;
    obs_.dof_vel = torch::zeros({1, params_.num_of_dofs});
    obs_.dof_vel_wheel = torch::zeros({1, 4});
    obs_.actions = torch::zeros({1, params_.num_of_dofs});
    obs_.height = torch::tensor({{0.5}});
    // Init output
    output_torques = torch::zeros({1, params_.num_of_dofs});
    output_dof_pos_ = params_.default_dof_pos;

    // Init control
    control_.vel_x = 0.0;
    control_.vel_y = 0.0;
    control_.vel_yaw = 0.0;
    control_.pos_x = 0.0;
    control_.pos_y = 0.0;
    control_.pos_z = 0.0;
    control_.pos_yaw = 0.0;
    control_.pos_roll = 3.14;
    control_.pos_pitch = 0.0;

    // history
    if (!params_.observations_history.empty()) {
        history_obs_buf_->clear();
    }

    running_ = true;
}

void StateQWRL::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    getState();
    if (!use_rl_thread_)
    {
        runModel();
    }
    setCommand();
}

void StateQWRL::exit()
{
    running_ = false;
}

FSMStateName StateQWRL::checkChange()
{
    if (enable_estimator_ and !estimator_->safety())
    {
        return FSMStateName::QWPASSIVE;
    }
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 0:
        return FSMStateName::QWPASSIVE;
    case 1:
        return FSMStateName::QWFIXEDDOWN;
    case 2:
        return FSMStateName::QWFIXEDSTAND;
    default:
        return FSMStateName::QWRL;
    }
}

torch::Tensor StateQWRL::computeObservation()
{
    std::vector<torch::Tensor> obs_list;

    for (const std::string& observation : params_.observations)
    {
        if (observation == "lin_vel")
        {
            obs_list.push_back(obs_.lin_vel * params_.lin_vel_scale);
        }
        else if (observation == "ang_vel")
        {
            obs_list.push_back(obs_.ang_vel * params_.ang_vel_scale);
        }
        else if (observation == "gravity_vec")
        {
            obs_list.push_back(quatRotateInverse(obs_.base_quat, obs_.gravity_vec, params_.framework));
        }
        else if (observation == "commands")
        {
            obs_list.push_back(obs_.commands);
        }
        else if (observation == "pose_commands")
        {
            obs_list.push_back(obs_.pose_commands); // scale TODO.
        }

        else if (observation == "dof_pos")
        {
            obs_list.push_back((obs_.dof_pos_leg - params_.default_dof_pos_leg) * params_.dof_pos_scale);
        }
        else if (observation == "dof_vel")
        {
            obs_list.push_back(obs_.dof_vel * params_.dof_vel_scale);
            // obs_list.push_back(obs_.dof_vel_wheel * params_.dof_vel_scale);
            // obs_list.push_back(obs_.dof_vel.slice(/*dim=*/0, /*start=*/12, /*end=*/16) * params_.dof_vel_scale);
        }
        else if (observation == "actions")
        {
            obs_list.push_back(obs_.actions);
        }
        else if (observation == "height")
        {
            obs_list.push_back(obs_.height);
        }
        
    }
    // for (int i = 0; i <obs_list.size(); i++)
    // {
    //     std::cout << obs_list[i].sizes() << std::endl;
    //     // obs_list[i].sizes;
    // }

    const torch::Tensor obs = cat(obs_list, 1);

    // std::cout << "Observation: " << obs << std::endl;
    torch::Tensor clamped_obs = clamp(obs, -params_.clip_obs, params_.clip_obs);
    return clamped_obs;
}

void StateQWRL::loadYaml(const std::string& config_path)
{
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path + "/config.yaml");
    }
    catch ([[maybe_unused]] YAML::BadFile& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("StateRL"), "The file '%s' does not exist", config_path.c_str());
        return;
    }

    params_.model_name = config["model_name"].as<std::string>();

    params_.model_name = config["model_name"].as<std::string>();
    params_.framework = config["framework"].as<std::string>();
    const int rows = config["rows"].as<int>();
    const int cols = config["cols"].as<int>();
    if (config["observations_history"].IsNull())
    {
        params_.observations_history = {};
    }
    else
    {
        params_.observations_history = ReadVectorFromYaml<int>(config["observations_history"]);
    }
    params_.decimation = config["decimation"].as<int>();
    params_.num_observations = config["num_observations"].as<int>();
    params_.observations = ReadVectorFromYaml<std::string>(config["observations"]);
    params_.clip_obs = config["clip_obs"].as<double>();
    if (config["clip_actions_lower"].IsNull() && config["clip_actions_upper"].IsNull())
    {
        params_.clip_actions_upper = torch::tensor({}).view({1, -1});
        params_.clip_actions_lower = torch::tensor({}).view({1, -1});
    }
    else
    {
        params_.clip_actions_upper = torch::tensor(
            ReadVectorFromYaml<double>(config["clip_actions_upper"], params_.framework, rows, cols)).view({1, -1});
        params_.clip_actions_lower = torch::tensor(
            ReadVectorFromYaml<double>(config["clip_actions_lower"], params_.framework, rows, cols)).view({1, -1});
    }
    params_.action_scale = config["action_scale"].as<double>();
    params_.action_scale_wheel = config["action_scale_wheel"].as<double>();
    params_.hip_scale_reduction = config["hip_scale_reduction"].as<double>();
    params_.hip_scale_reduction_indices = ReadVectorFromYaml<int>(config["hip_scale_reduction_indices"]);
    params_.num_of_dofs = config["num_of_dofs"].as<int>();
    params_.lin_vel_scale = config["lin_vel_scale"].as<double>();
    params_.ang_vel_scale = config["ang_vel_scale"].as<double>();
    params_.dof_pos_scale = config["dof_pos_scale"].as<double>();
    params_.dof_vel_scale = config["dof_vel_scale"].as<double>();
    // params_.commands_scale = torch::tensor(ReadVectorFromYaml<double>(config["commands_scale"])).view({1, -1});
    params_.commands_scale = torch::tensor({params_.lin_vel_scale, params_.lin_vel_scale, params_.ang_vel_scale});
    params_.rl_kp = torch::tensor(ReadVectorFromYaml<double>(config["rl_kp"], params_.framework, rows, cols)).view({
        1, -1
    });
    params_.rl_kd = torch::tensor(ReadVectorFromYaml<double>(config["rl_kd"], params_.framework, rows, cols)).view({
        1, -1
    });
    // params_.torque_limits = torch::tensor(
    //     ReadVectorFromYaml<double>(config["torque_limits"], params_.framework, rows, cols)).view({1, -1});

    params_.default_dof_pos = torch::from_blob(init_pos_, {params_.num_of_dofs}, torch::kDouble).clone().to(torch::kFloat).unsqueeze(0);
    params_.default_dof_pos_leg = torch::from_blob(init_pos_, {12}, torch::kDouble).clone().to(torch::kFloat).unsqueeze(0);

    // 18 policy
    for (int i = 0; i < params_.num_of_dofs; i++) {
        params_.default_dof_pos[0][i] = init_pos_[i];
    }

    for (int i = 0; i < 12; i++)
    {
        params_.default_dof_pos_leg[0][i] = init_pos_[i];
    }

    // 17 policy
    // for (int i = 0; i < params_.num_of_dofs; i++) {
    //     params_.default_dof_pos[0][i] = init_pos_[i];
    // }
    // for (int i = 0; i < 5; i++) {
    //     params_.default_dof_pos[0][i+12] = init_pos_[i+13];
    // }
    std::cout << "params_.default_dof_pos: " << params_.default_dof_pos << std::endl;
    // params_.default_dof_pos = torch::tensor(
    //     ReadVectorFromYaml<double>(config["default_dof_pos"], params_.framework, rows, cols)).view({1, -1});
}

torch::Tensor StateQWRL::quatRotateInverse(const torch::Tensor& q, const torch::Tensor& v, const std::string& framework)
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

// torch::Tensor StateQWRL::forward()
// {
//     // std::cout << "start forwarding!!!!!!!!!!!!!!! " << std::endl;
//     torch::autograd::GradMode::set_enabled(false);
//     torch::Tensor clamped_obs = computeObservation();
//     // std::cout << "the dimension of obs is"<< clamped_obs.sizes() << std::endl;
//     torch::Tensor actions;

//     if (!params_.observations_history.empty())
//     {
//         history_obs_buf_->insert(clamped_obs);
//         history_obs_ = history_obs_buf_->getObsVec(params_.observations_history);
//         actions = model_.forward({history_obs_}).toTensor();
//     }
//     else
//     {
//         actions = model_.forward({clamped_obs}).toTensor();
//     }

//     if (params_.clip_actions_upper.numel() != 0 && params_.clip_actions_lower.numel() != 0)
//     {
//         return clamp(actions, params_.clip_actions_lower, params_.clip_actions_upper);
//     }
//     return actions;
// }
// torch::Tensor StateQWRL::forward()
// {
//     torch::autograd::GradMode::set_enabled(false);
//     torch::Tensor clamped_obs = computeObservation();
//     torch::Tensor actions;
//
//     if (!params_.observations_history.empty())
//     {
//         history_obs_buf_->insert(clamped_obs);
//         history_obs_ = history_obs_buf_->getObsVec(params_.observations_history);
//         actions = model_.forward({history_obs_}).toTensor();
//     }
//     else
//     {
//         actions = model_.forward({clamped_obs}).toTensor();
//     }
//
//     return actions;
// }


torch::Tensor StateQWRL::forward()
{
    torch::NoGradGuard no_grad;

    // 1. 获取当前观测输入
    torch::Tensor clamped_obs = computeObservation();

    torch::Tensor input_obs;
    if (!params_.observations_history.empty())
    {
        history_obs_buf_->insert(clamped_obs);
        history_obs_ = history_obs_buf_->getObsVec(params_.observations_history);
        input_obs = history_obs_;
    }
    else
    {
        input_obs = clamped_obs;
    }

    // 2. 构造输入向量
    // [修正] 只传入 Observation，不传 Hidden State
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input_obs);

    // 3. 执行推理
    // 模型内部会自动读取并更新 self.hidden_state
    auto output_ivalue = model_.forward(inputs);

    // 4. 获取输出
    // [修正] 输出不再是 Tuple，而是直接的 Action Tensor
    torch::Tensor actions;
    if (output_ivalue.isTensor()) {
        actions = output_ivalue.toTensor();
    } else {
        // 防御性编程：以防万一未来导出的模型结构变了
        std::cerr << "[Error] Unexpected model output type. Expected Tensor." << std::endl;
    }

    return actions;
}

void StateQWRL::getState()
{
    if (params_.framework == "isaacgym")
    {
        robot_state_.imu.quaternion[3] = ctrl_interfaces_.imu_state_interface_[0].get().get_value();
        robot_state_.imu.quaternion[0] = ctrl_interfaces_.imu_state_interface_[1].get().get_value();
        robot_state_.imu.quaternion[1] = ctrl_interfaces_.imu_state_interface_[2].get().get_value();
        robot_state_.imu.quaternion[2] = ctrl_interfaces_.imu_state_interface_[3].get().get_value();
    }
    else if (params_.framework == "isaacsim")
    {
        robot_state_.imu.quaternion[0] = ctrl_interfaces_.imu_state_interface_[0].get().get_value();
        robot_state_.imu.quaternion[1] = ctrl_interfaces_.imu_state_interface_[1].get().get_value();
        robot_state_.imu.quaternion[2] = ctrl_interfaces_.imu_state_interface_[2].get().get_value();
        robot_state_.imu.quaternion[3] = ctrl_interfaces_.imu_state_interface_[3].get().get_value();
    }

    robot_state_.imu.gyroscope[0] = ctrl_interfaces_.imu_state_interface_[4].get().get_value();
    robot_state_.imu.gyroscope[1] = ctrl_interfaces_.imu_state_interface_[5].get().get_value();
    robot_state_.imu.gyroscope[2] = ctrl_interfaces_.imu_state_interface_[6].get().get_value();

    robot_state_.imu.accelerometer[0] = ctrl_interfaces_.imu_state_interface_[7].get().get_value();
    robot_state_.imu.accelerometer[1] = ctrl_interfaces_.imu_state_interface_[8].get().get_value();
    robot_state_.imu.accelerometer[2] = ctrl_interfaces_.imu_state_interface_[9].get().get_value();

    for (int i = 0; i < params_.num_of_dofs; i++)
    {
        robot_state_.motor_state.q[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        robot_state_.motor_state.dq[i] = ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value();
        robot_state_.motor_state.tauEst[i] = ctrl_interfaces_.joint_effort_state_interface_[i].get().get_value();
    }

    control_.vel_x = ctrl_interfaces_.control_inputs_.lx;
    control_.vel_y = ctrl_interfaces_.control_inputs_.ly;
    control_.vel_yaw = ctrl_interfaces_.control_inputs_.rx;

    control_.pos_x = ctrl_interfaces_.pose_cmd_inputs_.pos_x;
    control_.pos_y = ctrl_interfaces_.pose_cmd_inputs_.pos_y;
    control_.pos_z = ctrl_interfaces_.pose_cmd_inputs_.pos_z;
    control_.pos_roll = ctrl_interfaces_.pose_cmd_inputs_.pos_roll;
    control_.pos_pitch = ctrl_interfaces_.pose_cmd_inputs_.pos_pitch;
    control_.pos_yaw = ctrl_interfaces_.pose_cmd_inputs_.pos_yaw;

    updated_ = true;
}

void StateQWRL::runModel()
{
    if (enable_estimator_)
    {
        obs_.lin_vel = torch::from_blob(estimator_->getVelocity().data(), {3}, torch::kDouble).clone().
            to(torch::kFloat).unsqueeze(0);
    }
    obs_.ang_vel = torch::tensor(robot_state_.imu.gyroscope).unsqueeze(0);
    obs_.commands = torch::tensor({{control_.vel_x, control_.vel_y, control_.vel_yaw}});
    torch::Tensor ee_pos = torch::tensor({control_.pos_x, control_.pos_y, control_.pos_z}).unsqueeze(0);
    torch::Tensor ee_ori = EulartoQuat(torch::tensor({control_.pos_roll, control_.pos_pitch, control_.pos_yaw}));
    // RCLCPP_INFO(node_->get_logger(), "command ee pose are, x: %.3f, y: %.3f, z: %.3f, r: %.3f, p: %.3f, y: %.3f",
    //             control_.pos_x, control_.pos_y, control_.pos_z, control_.pos_roll, control_.pos_pitch,
    //             control_.pos_yaw);
    obs_.pose_commands = torch::cat({ee_pos, ee_ori}, 1);
    obs_.base_quat = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.base_quat = torch::tensor(robot_state_.imu.quaternion).unsqueeze(0);

    // 16 policy
    obs_.dof_pos = torch::tensor(robot_state_.motor_state.q).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);
    obs_.dof_pos_leg = torch::tensor(robot_state_.motor_state.q).narrow(0, 0, 12).unsqueeze(0);
    obs_.dof_vel = torch::tensor(robot_state_.motor_state.dq).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);
    obs_.dof_vel_wheel = torch::tensor(robot_state_.motor_state.dq).narrow(0, 12, 4).unsqueeze(0);
    obs_.height = torch::tensor({{ctrl_interfaces_.control_inputs_.ry}});
    // float v = control_.vel_yaw;                   // 假设是标量
    // v = std::clamp(v, 0.0f, 1.0f);
    // float vd = std::floor(v * 10.0f) / 10.0f;

    // // 若 obs_.height 需要 1D tensor
    // obs_.height = torch::tensor({{vd}});



    // 17 policy
    // obs_.dof_pos = torch::tensor(robot_state_.motor_state.q).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);
    // for (int i = 0; i < 5; i++) {
    //     obs_.dof_pos[0][i+12] = robot_state_.motor_state.q[i+13];
    // }
    // obs_.dof_vel = torch::tensor(robot_state_.motor_state.dq).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);
    // for (int i = 0; i < 5; i++) {
    //     obs_.dof_vel[0][i+12] = robot_state_.motor_state.dq[i+13];
    // }
    // const torch::Tensor clamped_actions = forward();

    // // const torch::Tensor clamped_actions_output = forward();
    // // torch::Tensor clamped_actions = clamped_actions_output.clone();
    // // for (int i = 0; i < 4; i++) {
    // //     clamped_actions[0][8+i] = clamped_actions_output[0][9+i];
    // // }
    // // clamped_actions[0][12] = clamped_actions_output[0][8];

    // obs_.actions = clamped_actions;
    // // obs_.actions = clamped_actions_output;

    // const torch::Tensor actions_scaled = clamped_actions * params_.action_scale;
    // // std::cout<<"actions_scaled before: "<<actions_scaled<<std::endl;
    // actions_scaled.slice(/*dim=*/1, /*start=*/0, /*end=*/12) = clamped_actions.slice(/*dim=*/1, /*start=*/0, /*end=*/12) * params_.action_scale;
    // actions_scaled.slice(/*dim=*/1, /*start=*/12, /*end=*/16) = clamped_actions.slice(/*dim=*/1, /*start=*/12, /*end=*/16) * params_.action_scale_wheel;
    // torch::Tensor joint_scale = torch::full({1, 12}, 0.25);
    // torch::Tensor wheel_scale = torch::full({1, 4}, 5.0);
    // actions_scaled.slice(/*dim=*/1, /*start=*/0, /*end=*/12) = clamped_actions.slice(/*dim=*/1, /*start=*/0, /*end=*/12) * joint_scale;
    // actions_scaled.slice(/*dim=*/1, /*start=*/12, /*end=*/16) = clamped_actions.slice(/*dim=*/1, /*start=*/12, /*end=*/16) * wheel_scale;
    // std::cout<<"actions_scaled after: "<<actions_scaled<<std::endl;
    // RCLCPP_INFO(node_->get_logger(),
    //             "action_scaled: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    //             actions_scaled[0][0], actions_scaled[0][1], actions_scaled[0][2], actions_scaled[0][3],
    //             actions_scaled[0][4], actions_scaled[0][5], actions_scaled[0][6], actions_scaled[0][7],
    //             actions_scaled[0][8], actions_scaled[0][9], actions_scaled[0][10], actions_scaled[0][11]);
    // std::cout << "obs_base_ang_vel: " << obs_.ang_vel << std::endl;
    // std::cout << "obs_joint_pose: " << obs_.dof_pos << std::endl;
    // std::cout << "obs_joint_vel: " << obs_.dof_vel << std::endl;
    // std::cout << "obs_actions: " << obs_.actions << std::endl;
    // std::cout << "obs_velocity_commands: " << obs_.commands << std::endl;
    // std::cout << "obs_pose_command: " << obs_.pose_commands << std::endl;
    // std::cout << "obs_projected_gravity: " << obs_.gravity_vec << std::endl;
    // std::cout << "obs_actions: " << obs_.actions << std::endl;

    // torch::Tensor output_torques = params_.rl_kp * (actions_scaled + params_.default_dof_pos - obs_.dof_pos) - params_.rl_kd * obs_.dof_vel;
    torch::Tensor raw_actions = forward();

    // step1: scale
    torch::Tensor actions_scaled = raw_actions.clone();
    actions_scaled.slice(/*dim=*/1, /*start=*/0, /*end=*/12) = raw_actions.slice(/*dim=*/1, /*start=*/0, /*end=*/12) * params_.action_scale;
    actions_scaled.slice(/*dim=*/1, /*start=*/12, /*end=*/16) = raw_actions.slice(/*dim=*/1, /*start=*/12, /*end=*/16) * params_.action_scale_wheel;

    // step2: clip
    if (params_.clip_actions_upper.numel() != 0 && params_.clip_actions_lower.numel() != 0)
    {
        actions_scaled = clamp(actions_scaled, params_.clip_actions_lower, params_.clip_actions_upper);
    }
    torch::Tensor actions_raw_scaled = raw_actions.clone();
    if (params_.clip_actions_upper.numel() != 0 && params_.clip_actions_lower.numel() != 0)
    {
        actions_raw_scaled = clamp(actions_raw_scaled, params_.clip_actions_lower, params_.clip_actions_upper);
    }
    obs_.actions = actions_raw_scaled;
    output_dof_pos_ = actions_scaled + params_.default_dof_pos;


    for (int i = 0; i < params_.num_of_dofs - 4; ++i)
    {
        robot_command_.motor_command.q[i] = output_dof_pos_[0][i].item<double>();
        robot_command_.motor_command.dq[i] = 0;
        robot_command_.motor_command.kp[i] = params_.rl_kp[0][i].item<double>();
        robot_command_.motor_command.kd[i] = params_.rl_kd[0][i].item<double>();
        robot_command_.motor_command.tau[i] = 0;
    }
    //
    for (int i = 0; i < 4; ++i)
    {
        robot_command_.motor_command.q[i + params_.num_of_dofs - 4] = 0;
        robot_command_.motor_command.dq[i + params_.num_of_dofs - 4] = output_dof_pos_[0][i + params_.num_of_dofs - 4].item<double>();
        robot_command_.motor_command.kp[i + params_.num_of_dofs - 4] = 0.0;
        robot_command_.motor_command.kd[i + params_.num_of_dofs - 4] = 3.0;
        robot_command_.motor_command.tau[i + params_.num_of_dofs - 4] = 0;
    }


    // std::cout << "command q are: " << robot_command_.motor_command.q << std::endl;
}

void StateQWRL::setCommand() const
{
    for (int i = 0; i < params_.num_of_dofs; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(robot_command_.motor_command.q[i]);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(robot_command_.motor_command.dq[i]);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(robot_command_.motor_command.kp[i]);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(robot_command_.motor_command.kd[i]);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(robot_command_.motor_command.tau[i]);
    }
}


torch::Tensor StateQWRL::EulartoQuat(torch::Tensor euler) {
    // euler: [roll, pitch, yaw]
    double roll = euler[0].item<double>();
    double pitch = euler[1].item<double>();
    double yaw = euler[2].item<double>();

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    torch::Tensor q = torch::zeros(4);
    q[0] = cr * cp * cy + sr * sp * sy; // w
    q[1] = sr * cp * cy - cr * sp * sy; // x
    q[2] = cr * sp * cy + sr * cp * sy; // y
    q[3] = cr * cp * sy - sr * sp * cy; // z

    return q.unsqueeze(0);
}
