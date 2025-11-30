//
// Created by ray on 2025-07-25.
//

#include "rl_quadruped_manipulation_controller/FSM/StateQMRL.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>

template<typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node &node) {
    std::vector<T> values;
    for (const auto &val: node) {
        values.push_back(val.as<T>());
    }
    return values;
}

template<typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node &node, const std::string &framework, const int &rows,
                                  const int &cols) {
    std::vector<T> values;
    for (const auto &val: node) {
        values.push_back(val.as<T>());
    }

    if (framework == "isaacsim") {
        std::vector<T> transposed_values(cols * rows);
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                transposed_values[c * rows + r] = values[r * cols + c];
            }
        }
        return transposed_values;
    }
    if (framework == "isaacgym") {
        return values;
    }
    throw std::invalid_argument("Unsupported framework: " + framework);
}

StateQMRL::StateQMRL(CtrlInterfaces &ctrl_interfaces,
                     CtrlComponent &ctrl_component,
                     const std::vector<double> &target_pos) : FSMState(FSMStateName::QMRL, "qm rl", ctrl_interfaces),
                                                              node_(ctrl_component.node_),
                                                              enable_estimator_(ctrl_component.enable_estimator_),
                                                              estimator_(ctrl_component.estimator_) {
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

    for (int i = 0; i < 20; i++) {
        init_pos_[i] = target_pos[i];
    }
    // read params from yaml
    loadYaml(model_path);

    if (!params_.observations_history.empty()) {
        history_obs_buf_ = std::make_shared<ObservationBuffer>(1, params_.num_observations,
                                                               params_.observations_history.size());
    }

    RCLCPP_INFO(node_->get_logger(), "Model loading: %s", params_.model_name.c_str());
    model_ = torch::jit::load(model_path + "/" + params_.model_name);

    if (use_rl_thread_) {
        rl_thread_ = std::thread([&] {
            while (true) {
                try {
                    executeAndSleep(
                        [&] {
                            if (running_) {
                                runModel();
                            }
                        },
                        ctrl_interfaces_.frequency_ / params_.decimation);
                } catch (const std::exception &e) {
                    running_ = false;
                    RCLCPP_ERROR(rclcpp::get_logger("StateRL"), "Error in RL thread: %s", e.what());
                }
            }
        });
        setThreadPriority(60, rl_thread_);
    }
}

void StateQMRL::enter() {
    // Init observations
    obs_.lin_vel = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.ang_vel = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    obs_.commands = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.pose_commands = torch::tensor({
        {
            0.55, -0.032649196684360504, 0.12, -0.014631232246756554, 0.013575111515820026, 0.9998008012771606,
            0.00019866018556058407
        }
    });
    obs_.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}});
    obs_.dof_pos = params_.default_dof_pos;
    obs_.dof_vel = torch::zeros({1, params_.num_of_dofs});
    obs_.actions = torch::zeros({1, params_.num_of_dofs});

    // Init output
    output_torques = torch::zeros({1, params_.num_of_dofs});
    output_dof_pos_ = params_.default_dof_pos;

    // Init control
    control_.vel_x = 0.0;
    control_.vel_y = 0.0;
    control_.vel_yaw = 0.0;
    control_.pos_x = 0.55;
    control_.pos_y = 0.0;
    control_.pos_z = 0.10;
    control_.pos_yaw = 0.0;
    control_.pos_roll = 3.14;
    control_.pos_pitch = 0.0;

    // history
    if (!params_.observations_history.empty()) {
        history_obs_buf_->clear();
    }

    running_ = true;
}

void StateQMRL::run(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
    getState();
    if (!use_rl_thread_) {
        runModel();
    }
    setCommand();
}

void StateQMRL::exit() {
    running_ = false;
}

FSMStateName StateQMRL::checkChange() {
    if (enable_estimator_ and !estimator_->safety()) {
        return FSMStateName::QMPASSIVE;
    }
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 0:
            return FSMStateName::QMPASSIVE;
        case 1:
            return FSMStateName::QMFIXEDDOWN;
        case 3:
            return FSMStateName::QMFIXEDSTAND;
        default:
            return FSMStateName::QMRL;
    }
}

torch::Tensor StateQMRL::computeObservation() {
    std::vector<torch::Tensor> obs_list;

    for (const std::string &observation: params_.observations) {
        if (debug_ == 1) {
            if (observation == "lin_vel") {
                obs_list.push_back(obs_.lin_vel);
            } else if (observation == "ang_vel") {
                obs_list.push_back(obs_.ang_vel);
            } else if (observation == "gravity_vec") {
                obs_list.push_back(obs_.gravity_vec);
            } else if (observation == "vel_commands") {
                obs_list.push_back(obs_.commands);
            } else if (observation == "pose_commands") {
                obs_list.push_back(obs_.pose_commands); // scale TODO.
            } else if (observation == "dof_pos") {
                obs_list.push_back(obs_.dof_pos);
            } else if (observation == "dof_vel") {
                obs_list.push_back(obs_.dof_vel);
            } else if (observation == "actions") {
                obs_list.push_back(obs_.actions);
            }
        } else {
            if (observation == "lin_vel") {
                obs_list.push_back(obs_.lin_vel * params_.lin_vel_scale);
            } else if (observation == "ang_vel") {
                obs_list.push_back(obs_.ang_vel * params_.ang_vel_scale);
            } else if (observation == "gravity_vec") {
                obs_list.push_back(quatRotateInverse(obs_.base_quat, obs_.gravity_vec, params_.framework));
            } else if (observation == "vel_commands") {
                obs_list.push_back(obs_.commands);
            } else if (observation == "pose_commands") {
                obs_list.push_back(obs_.pose_commands); // scale TODO.
            } else if (observation == "dof_pos") {
                obs_list.push_back((obs_.dof_pos - params_.default_dof_pos) * params_.dof_pos_scale);
            } else if (observation == "dof_vel") {
                obs_list.push_back(obs_.dof_vel * params_.dof_vel_scale);
            } else if (observation == "actions") {
                obs_list.push_back(obs_.actions);
            }
        }
    }

    const torch::Tensor obs = cat(obs_list, 1);

    // std::cout << "Observation: " << obs << std::endl;
    torch::Tensor clamped_obs = clamp(obs, -params_.clip_obs, params_.clip_obs);
    return clamped_obs;
}

void StateQMRL::loadYaml(const std::string &config_path) {
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_path + "/config_qm.yaml");
    } catch ([[maybe_unused]] YAML::BadFile &e) {
        RCLCPP_ERROR(rclcpp::get_logger("StateRL"), "The file '%s' does not exist", config_path.c_str());
        return;
    }

    params_.model_name = config["model_name"].as<std::string>();

    params_.model_name = config["model_name"].as<std::string>();
    params_.framework = config["framework"].as<std::string>();
    const int rows = config["rows"].as<int>();
    const int cols = config["cols"].as<int>();
    debug_ = config["debug"].as<int>();
    if (config["observations_history"].IsNull()) {
        params_.observations_history = {};
    } else {
        params_.observations_history = ReadVectorFromYaml<int>(config["observations_history"]);
    }
    params_.decimation = config["decimation"].as<int>();
    params_.num_observations = config["num_observations"].as<int>();
    params_.observations = ReadVectorFromYaml<std::string>(config["observations"]);
    params_.clip_obs = config["clip_obs"].as<double>();
    if (config["clip_actions_lower"].IsNull() && config["clip_actions_upper"].IsNull()) {
        params_.clip_actions_upper = torch::tensor({}).view({1, -1});
        params_.clip_actions_lower = torch::tensor({}).view({1, -1});
    } else {
        params_.clip_actions_upper = torch::tensor(
            ReadVectorFromYaml<double>(config["clip_actions_upper"], params_.framework, rows, cols)).view({1, -1});
        params_.clip_actions_lower = torch::tensor(
            ReadVectorFromYaml<double>(config["clip_actions_lower"], params_.framework, rows, cols)).view({1, -1});
    }
    params_.action_scale = config["action_scale"].as<double>();
    // params_.action_scales = torch::tensor(ReadVectorFromYaml<double>(config["action_scales"], params_.framework, rows, cols)).view({
    //     1, -1
    // });
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
    params_.torque_limits = torch::tensor(
        ReadVectorFromYaml<double>(config["torque_limits"], params_.framework, rows, cols)).view({1, -1});

    params_.default_dof_pos = torch::from_blob(init_pos_, {params_.num_of_dofs}, torch::kDouble).clone().
            to(torch::kFloat).unsqueeze(0);

    // 18 policy
    for (int i = 0; i < params_.num_of_dofs; i++) {
        params_.default_dof_pos[0][i] = init_pos_[i];
    }
}

torch::Tensor StateQMRL::quatRotateInverse(const torch::Tensor &q, const torch::Tensor &v,
                                           const std::string &framework) {
    torch::Tensor q_w;
    torch::Tensor q_vec;
    if (framework == "isaacsim") {
        q_w = q.index({torch::indexing::Slice(), 0});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(1, 4)});
    } else if (framework == "isaacgym") {
        q_w = q.index({torch::indexing::Slice(), 3});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});
    }
    const c10::IntArrayRef shape = q.sizes();

    const torch::Tensor a = v * (2.0 * torch::pow(q_w, 2) - 1.0).unsqueeze(-1);
    const torch::Tensor b = cross(q_vec, v, -1) * q_w.unsqueeze(-1) * 2.0;
    const torch::Tensor c = q_vec * bmm(q_vec.view({shape[0], 1, 3}), v.view({shape[0], 3, 1})).squeeze(-1) * 2.0;
    return a - b + c;
}

torch::Tensor StateQMRL::forward() {
    // std::cout << "start forwarding!!!!!!!!!!!!!!! " << std::endl;
    torch::autograd::GradMode::set_enabled(false);
    torch::Tensor clamped_obs = computeObservation();
    torch::Tensor actions;

    if (!params_.observations_history.empty()) {
        // printf("obs_history!!!!!!!!");
        history_obs_buf_->insert(clamped_obs);
        history_obs_ = history_obs_buf_->getObsVec(params_.observations_history);
        actions = model_.forward({history_obs_}).toTensor();
    } else {
        actions = model_.forward({clamped_obs}).toTensor();
    }

    if (params_.clip_actions_upper.numel() != 0 && params_.clip_actions_lower.numel() != 0) {
        return clamp(actions, params_.clip_actions_lower, params_.clip_actions_upper);
    }
    return actions;
}

void StateQMRL::getState() {
    if (params_.framework == "isaacgym") {
        robot_state_.imu.quaternion[3] = ctrl_interfaces_.imu_state_interface_[0].get().get_value();
        robot_state_.imu.quaternion[0] = ctrl_interfaces_.imu_state_interface_[1].get().get_value();
        robot_state_.imu.quaternion[1] = ctrl_interfaces_.imu_state_interface_[2].get().get_value();
        robot_state_.imu.quaternion[2] = ctrl_interfaces_.imu_state_interface_[3].get().get_value();
    } else if (params_.framework == "isaacsim") {
        robot_state_.imu.quaternion[0] = ctrl_interfaces_.imu_state_interface_[0].get().get_value(); //w
        robot_state_.imu.quaternion[1] = ctrl_interfaces_.imu_state_interface_[1].get().get_value(); //x
        robot_state_.imu.quaternion[2] = ctrl_interfaces_.imu_state_interface_[2].get().get_value(); //y
        robot_state_.imu.quaternion[3] = ctrl_interfaces_.imu_state_interface_[3].get().get_value(); //z
    }

    robot_state_.imu.gyroscope[0] = ctrl_interfaces_.imu_state_interface_[4].get().get_value();
    robot_state_.imu.gyroscope[1] = ctrl_interfaces_.imu_state_interface_[5].get().get_value();
    robot_state_.imu.gyroscope[2] = ctrl_interfaces_.imu_state_interface_[6].get().get_value();

    robot_state_.imu.accelerometer[0] = ctrl_interfaces_.imu_state_interface_[7].get().get_value();
    robot_state_.imu.accelerometer[1] = ctrl_interfaces_.imu_state_interface_[8].get().get_value();
    robot_state_.imu.accelerometer[2] = ctrl_interfaces_.imu_state_interface_[9].get().get_value();

    for (int i = 0; i < 20; i++) {
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

void StateQMRL::runModel() {
    if (debug_ == 1) {
        obs_.actions = torch::tensor({
            {
                -0.6379608511924744, 0.6532260775566101, 0.04194433614611626, -0.10111343860626221, -0.9810376763343811,
                -0.47113335132598877, 0.47732415795326233, 0.15141227841377258, 1.0485986471176147, 1.4754494428634644,
                -0.8462913036346436, -0.7575194835662842, 0.28035011887550354, -0.9920223951339722, -1.1165214776992798,
                -0.08590316772460938, -0.27153506875038147, -0.31173741817474365
            }
        });
        auto action_cpu = obs_.actions.to(torch::kCPU);
        auto action_a = action_cpu.accessor<float, 2>();
        RCLCPP_INFO(node_->get_logger(),
                    "obs_action: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                    action_a[0][0], // 第一行第一列
                    action_a[0][1], // 第一行第二列
                    action_a[0][2],
                    action_a[0][3], // 第一行第一列
                    action_a[0][4], // 第一行第二列
                    action_a[0][5],
                    action_a[0][6], // 第一行第一列
                    action_a[0][7], // 第一行第二列
                    action_a[0][8],
                    action_a[0][9], // 第一行第一列
                    action_a[0][10], // 第一行第二列
                    action_a[0][11],
                    action_a[0][12], // 第一行第一列
                    action_a[0][13], // 第一行第二列
                    action_a[0][14],
                    action_a[0][15], // 第一行第一列
                    action_a[0][16], // 第一行第二列
                    action_a[0][17]); // 第一行第三列
        obs_.ang_vel = torch::tensor({{-0.007297192700207233, -0.001976228319108486, 0.0015291562303900719}});
        obs_.commands = torch::tensor({{control_.vel_x, control_.vel_y, control_.vel_yaw}});
        obs_.pose_commands = torch::tensor({
            {
                0.6355149149894714, 0.04724014550447464, 0.05160874128341675, -0.00383570184931159,
                -0.07444332540035248, 0.9968289136886597, -0.02785065397620201
            }
        });

        obs_.dof_pos = torch::tensor({
            {
                0.011285459622740746, -0.03838716447353363, 0.02198888175189495, 0.05333109200000763,
                0.07875794172286987, 0.0424426794052124, -0.002448558807373047, 0.0838160514831543,
                -0.10473048686981201, -0.08744251728057861, 0.020848631858825684, 0.05955231189727783,
                0.10515580326318741, 0.3715236186981201, -0.136971116065979, -0.05798939988017082, -0.15991264581680298,
                -0.07725618779659271
            }
        });

        obs_.dof_vel = torch::tensor({
            {
                0.002173038199543953, 0.0018886991310864687, 0.002735392190515995, 0.0027210921980440617,
                0.0033251445274800062, 0.0017088508466258645, -0.004240454640239477, -0.00013552144810091704,
                -0.006636509206146002, -0.003172545228153467, 0.0065030804835259914, -4.638778045773506e-05,
                0.0028095131274312735, 0.005108881276100874, 0.0015534991398453712, 0.001577503397129476,
                0.0042191483080387115, -0.1086830422282219
            }
        });

        obs_.gravity_vec = torch::tensor({{0.025889592245221138, 0.00028420519083738327, -0.9996647834777832}});

        const torch::Tensor clamped_actions = forward();
        for (const int i: params_.hip_scale_reduction_indices) {
            clamped_actions[0][i] *= params_.hip_scale_reduction;
        }

        obs_.actions = clamped_actions;

        auto ang_vel_cpu = obs_.ang_vel.to(torch::kCPU);
        auto project_gravity_cpu = obs_.gravity_vec.to(torch::kCPU);
        auto vel_command_cpu = obs_.commands.to(torch::kCPU);
        auto joint_pose_rel_cpu = obs_.dof_pos.to(torch::kCPU);
        auto joint_vel_cpu = obs_.dof_vel.to(torch::kCPU);
        auto action_output_cpu = obs_.actions.to(torch::kCPU);
        auto pose_command_cup = obs_.pose_commands.to(torch::kCPU);

        // 获取访问器（2D 张量）
        auto ang_vel_a = ang_vel_cpu.accessor<float, 2>();
        auto project_gravity_a = project_gravity_cpu.accessor<float, 2>();
        auto vel_command_a = vel_command_cpu.accessor<float, 2>();
        auto joint_pose_rel_a = joint_pose_rel_cpu.accessor<float, 2>();
        auto joint_vel_a = joint_vel_cpu.accessor<float, 2>();
        auto action__output_a = action_output_cpu.accessor<float, 2>();
        auto pose_command_a = pose_command_cup.accessor<float, 2>();

        // 打印值
        RCLCPP_INFO(node_->get_logger(),
                    "obs_base_ang_vel: %.3f, %.3f, %.3f",
                    ang_vel_a[0][0], // 第一行第一列
                    ang_vel_a[0][1], // 第一行第二列
                    ang_vel_a[0][2]); // 第一行第三列
        RCLCPP_INFO(node_->get_logger(),
                    "obs_project_gravity: %.3f, %.3f, %.3f",
                    project_gravity_a[0][0], // 第一行第一列
                    project_gravity_a[0][1], // 第一行第二列
                    project_gravity_a[0][2]); // 第一行第三列
        RCLCPP_INFO(node_->get_logger(),
                    "obs_veloity command: %.3f, %.3f, %.3f",
                    vel_command_a[0][0], // 第一行第一列
                    vel_command_a[0][1], // 第一行第二列
                    vel_command_a[0][2]); // 第一行第三列

        RCLCPP_INFO(node_->get_logger(),
                    "obs_joint pose_rel: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                    joint_pose_rel_a[0][0], // 第一行第一列
                    joint_pose_rel_a[0][1], // 第一行第二列
                    joint_pose_rel_a[0][2],
                    joint_pose_rel_a[0][3], // 第一行第一列
                    joint_pose_rel_a[0][4], // 第一行第二列
                    joint_pose_rel_a[0][5],
                    joint_pose_rel_a[0][6], // 第一行第一列
                    joint_pose_rel_a[0][7], // 第一行第二列
                    joint_pose_rel_a[0][8],
                    joint_pose_rel_a[0][9], // 第一行第一列
                    joint_pose_rel_a[0][10], // 第一行第二列
                    joint_pose_rel_a[0][11],
                    joint_pose_rel_a[0][12], // 第一行第一列
                    joint_pose_rel_a[0][13], // 第一行第二列
                    joint_pose_rel_a[0][14],
                    joint_pose_rel_a[0][15], // 第一行第一列
                    joint_pose_rel_a[0][16], // 第一行第二列
                    joint_pose_rel_a[0][17]); // 第一行第三列

        RCLCPP_INFO(node_->get_logger(),
                    "obs_joint_vel: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                    joint_vel_a[0][0], // 第一行第一列
                    joint_vel_a[0][1], // 第一行第二列
                    joint_vel_a[0][2],
                    joint_vel_a[0][3], // 第一行第一列
                    joint_vel_a[0][4], // 第一行第二列
                    joint_vel_a[0][5],
                    joint_vel_a[0][6], // 第一行第一列
                    joint_vel_a[0][7], // 第一行第二列
                    joint_vel_a[0][8],
                    joint_vel_a[0][9], // 第一行第一列
                    joint_vel_a[0][10], // 第一行第二列
                    joint_vel_a[0][11],
                    joint_vel_a[0][12], // 第一行第一列
                    joint_vel_a[0][13], // 第一行第二列
                    joint_vel_a[0][14],
                    joint_vel_a[0][15], // 第一行第一列
                    joint_vel_a[0][16], // 第一行第二列
                    joint_vel_a[0][17]); // 第一行第三列

        RCLCPP_INFO(node_->get_logger(),
                    "obs_pose_command: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                    pose_command_a[0][0], // 第一行第一列
                    pose_command_a[0][1], // 第一行第二列
                    pose_command_a[0][2],
                    pose_command_a[0][3], // 第一行第一列
                    pose_command_a[0][4], // 第一行第二列
                    pose_command_a[0][5],
                    pose_command_a[0][6]); // 第一行第三列

        RCLCPP_INFO(node_->get_logger(),
                    "action_output: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                    action__output_a[0][0], // 第一行第一列
                    action__output_a[0][1], // 第一行第二列
                    action__output_a[0][2],
                    action__output_a[0][3], // 第一行第一列
                    action__output_a[0][4], // 第一行第二列
                    action__output_a[0][5],
                    action__output_a[0][6], // 第一行第一列
                    action__output_a[0][7], // 第一行第二列
                    action__output_a[0][8],
                    action__output_a[0][9], // 第一行第一列
                    action__output_a[0][10], // 第一行第二列
                    action__output_a[0][11],
                    action__output_a[0][12], // 第一行第一列
                    action__output_a[0][13], // 第一行第二列
                    action__output_a[0][14],
                    action__output_a[0][15], // 第一行第一列
                    action__output_a[0][16], // 第一行第二列
                    action__output_a[0][17]); // 第一行第三列
        std::cout << "================================================" << std::endl;
        // std::cout << "default_dof_pos: " << params_.default_dof_pos << std::endl;
        // std::cout << "output_dof_pos_: " << output_dof_pos_ << std::endl;
        // output_dof_pos_ =  params_.default_dof_pos;
        for (int i = 0; i < 18; ++i) {
            robot_command_.motor_command.q[i] = init_pos_[i];
            robot_command_.motor_command.dq[i] = 0;
            robot_command_.motor_command.kp[i] = 100.0;
            robot_command_.motor_command.kd[i] = 3.0;
            robot_command_.motor_command.tau[i] = 0;
        }
    } else {
        if (enable_estimator_) {
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
        obs_.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}});
        obs_.base_quat = torch::tensor(robot_state_.imu.quaternion).unsqueeze(0);

        // 18 policy
        obs_.dof_pos = torch::tensor(robot_state_.motor_state.q).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);
        obs_.dof_vel = torch::tensor(robot_state_.motor_state.dq).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);
        const torch::Tensor clamped_actions = forward();
        for (const int i: params_.hip_scale_reduction_indices) {
            clamped_actions[0][i] *= params_.hip_scale_reduction;
        }

        obs_.actions = clamped_actions;
        // obs_.actions = clamped_actions_output;

        // const torch::Tensor actions_scaled = clamped_actions * params_.action_scales;
        const torch::Tensor actions_scaled = clamped_actions * params_.action_scale;

        // torch::Tensor output_torques = params_.rl_kp * (actions_scaled + params_.default_dof_pos - obs_.dof_pos) - params_.rl_kd * obs_.dof_vel;
        // output_torques = clamp(output_torques, -(params_.torque_limits), params_.torque_limits);

        output_dof_pos_ = actions_scaled + params_.default_dof_pos;
        for (int i = 0; i < params_.num_of_dofs; ++i) {
            robot_command_.motor_command.q[i] = output_dof_pos_[0][i].item<double>();
            robot_command_.motor_command.dq[i] = 0;
            robot_command_.motor_command.kp[i] = params_.rl_kp[0][i].item<double>();
            robot_command_.motor_command.kd[i] = params_.rl_kd[0][i].item<double>();
            robot_command_.motor_command.tau[i] = 0;
        }
    }
    // std::cout << "command q are: " << robot_command_.motor_command.q << std::endl;
}

void StateQMRL::setCommand() const {
    for (int i = 0; i < params_.num_of_dofs + 2; i++) {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(robot_command_.motor_command.q[i]);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(robot_command_.motor_command.dq[i]);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(robot_command_.motor_command.kp[i]);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(robot_command_.motor_command.kd[i]);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(robot_command_.motor_command.tau[i]);
    }
}


torch::Tensor StateQMRL::EulartoQuat(torch::Tensor euler) {
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
