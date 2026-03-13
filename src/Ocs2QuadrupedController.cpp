#include "Ocs2QuadrupedController.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_quadruped_controller/estimator/LinearKalmanFilter.h>
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
#include <ocs2_quadruped_controller/wbc/HierarchicalWbc.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_sqp/SqpMpc.h>
#include <angles/angles.h>
#include <ocs2_quadruped_controller/control/GaitManager.h>

#define LOG_TRACE(msg) \
{ \
  FILE* _f = fopen("/tmp/ocs2_crash_trace.txt", "a"); \
  if(_f) { fprintf(_f, "%s\n", msg); fclose(_f); } \
}

namespace ocs2::legged_robot
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration Ocs2QuadrupedController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto &joint_name : joint_names_)
        {
            for (const auto &interface_type : command_interface_types_)
            {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration Ocs2QuadrupedController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name : joint_names_)
        {
            for (const auto &interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        for (const auto &interface_type : imu_interface_types_)
        {
            conf.names.push_back(imu_name_ + "/" += interface_type);
        }

        // for (const auto &interface_type : foot_force_interface_types_)
        // {
        //     conf.names.push_back(foot_force_name_ + "/" += interface_type);
        // }

        return conf;
    }

    controller_interface::return_type Ocs2QuadrupedController::update(const rclcpp::Time &time,
                                                                      const rclcpp::Duration &period)
    {
        // State Estimate
        updateStateEstimation(modeNumber2StanceLeg(planned_mode), time, period);

        // Terrain Estimator
        terrain_estimator_->update(ctrl_comp_.observation_.state.segment(9, 3),
                                   ctrl_comp_.estimator_->getPoseFeet2Body(),
                                   modeNumber2StanceLeg(planned_mode));

        // Compute target trajectory
        ctrl_comp_.target_manager_->update(terrain_estimator_->getGroundEulerAngleWrtBody(), time, period);

        // Update the current state of the system
        mpc_mrt_interface_->setCurrentObservation(ctrl_comp_.observation_);

        // Load the latest MPC policy
        mpc_mrt_interface_->updatePolicy();

        // Evaluate the current policy
        vector_t optimized_state, optimized_input;
        mpc_mrt_interface_->evaluatePolicy(ctrl_comp_.observation_.time, ctrl_comp_.observation_.state, optimized_state,
                                           optimized_input, planned_mode);

        // Whole body control
        ctrl_comp_.observation_.input = optimized_input;

        wbc_timer_.startTimer();
        vector_t x = wbc_->update(optimized_state, optimized_input, measured_rbd_state_, planned_mode, period.seconds());
        wbc_timer_.endTimer();

        vector_t torque = x.tail(legged_interface_->getCentroidalModelInfo().actuatedDofNum);

        vector_t pos_des = centroidal_model::getJointAngles(optimized_state,
                                                            legged_interface_->getCentroidalModelInfo());
        vector_t vel_des = centroidal_model::getJointVelocities(optimized_input,
                                                                legged_interface_->getCentroidalModelInfo());

        // Safety check, if failed, stop the controller
        if (!safety_checker_->check(ctrl_comp_.observation_, optimized_state, optimized_input, ctrl_comp_))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "[Legged Controller] Safety check failed, stopping the controller.");
            for (int i = 0; i < joint_names_.size(); i++)
            {
                ctrl_comp_.joint_torque_command_interface_[i].get().set_value(0);
                ctrl_comp_.joint_position_command_interface_[i].get().set_value(0);
                ctrl_comp_.joint_velocity_command_interface_[i].get().set_value(0);
                ctrl_comp_.joint_kp_command_interface_[i].get().set_value(0.0);
                ctrl_comp_.joint_kd_command_interface_[i].get().set_value(5);
            }
            return controller_interface::return_type::ERROR;
        }

        size_t actuated_dof_num = legged_interface_->getCentroidalModelInfo().actuatedDofNum;
        for (int i = 0; i < joint_names_.size(); i++)
        {
            if (i < actuated_dof_num) {
                ctrl_comp_.joint_torque_command_interface_[i].get().set_value(torque(i));
                ctrl_comp_.joint_position_command_interface_[i].get().set_value(pos_des(i));
                ctrl_comp_.joint_velocity_command_interface_[i].get().set_value(vel_des(i));
                ctrl_comp_.joint_kp_command_interface_[i].get().set_value(default_kp_);
                ctrl_comp_.joint_kd_command_interface_[i].get().set_value(default_kd_);
            } else {
                // Unactuated box joints (passive springs) or extra joints
                ctrl_comp_.joint_torque_command_interface_[i].get().set_value(0.0);
                // Keep the box joint steady at 0.12 (the resting length in the stand_controller)
                ctrl_comp_.joint_position_command_interface_[i].get().set_value(0.12);
                ctrl_comp_.joint_velocity_command_interface_[i].get().set_value(0.0);
                ctrl_comp_.joint_kp_command_interface_[i].get().set_value(250.0);
                ctrl_comp_.joint_kd_command_interface_[i].get().set_value(25.0);
            }
        }

        // // Visualization
        // ctrl_comp_.visualizer_->update(ctrl_comp_.observation_, mpc_mrt_interface_->getPolicy(),
        //                                mpc_mrt_interface_->getCommand());

        observation_publisher_->publish(ros_msg_conversions::createObservationMsg(ctrl_comp_.observation_));

        // /******************************************位置环复位******************************************/
        // updateStateEstimation(modeNumber2StanceLeg(planned_mode), time, period);
        // std::vector<double> target_position(12, 0.0);
        // for (int i = 0; i < joint_names_.size(); i++)
        // {
        //     target_position[i] = stand_controller_->calcTargetPosition(i, ctrl_comp_.observation_.time, 5.0);
        //
        //     ctrl_comp_.joint_torque_command_interface_[i].get().set_value(0);
        //     ctrl_comp_.joint_position_command_interface_[i].get().set_value(target_position[i]);
        //     ctrl_comp_.joint_velocity_command_interface_[i].get().set_value(0);
        //     ctrl_comp_.joint_kp_command_interface_[i].get().set_value(250);
        //     ctrl_comp_.joint_kd_command_interface_[i].get().set_value(25);
        //
        // }
        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_init()
    {
        // Initialize OCS2
        urdf_file_ = auto_declare<std::string>("urdf_file", urdf_file_);
        task_file_ = auto_declare<std::string>("task_file", task_file_);
        reference_file_ = auto_declare<std::string>("reference_file", reference_file_);
        gait_file_ = auto_declare<std::string>("gait_file", gait_file_);

        get_node()->get_parameter("update_rate", ctrl_comp_.frequency_);
        RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_comp_.frequency_);

        // Load verbose parameter from the task file
        verbose_ = false;
        loadData::loadCppDataType(task_file_, "legged_robot_interface.verbose", verbose_);

        // Hardware Parameters
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        feet_names_ = auto_declare<std::vector<std::string>>("feet", feet_names_);
        command_interface_types_ =
            auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        state_interface_types_ =
            auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
        imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
        imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", state_interface_types_);
        // foot_force_name_ = auto_declare<std::string>("foot_force_name", foot_force_name_);
        // foot_force_interface_types_ =
        //     auto_declare<std::vector<std::string>>("foot_force_interfaces", state_interface_types_);

        // PD gains
        default_kp_ = auto_declare<double>("default_kp", default_kp_);
        default_kd_ = auto_declare<double>("default_kd", default_kd_);

        setupLeggedInterface();
        setupMpc();
        setupMrt();

        CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
        eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
            legged_interface_->getPinocchioInterface(), pinocchio_mapping,
            legged_interface_->modelSettings().contactNames3DoF);

        // // Visualization
        // ctrl_comp_.visualizer_ = std::make_shared<LeggedRobotVisualizer>(
        //     legged_interface_->getPinocchioInterface(), legged_interface_->getCentroidalModelInfo(), *eeKinematicsPtr_,
        //     get_node());

        // selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
        //                                                                        leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

        // State estimation
        planned_mode = 0;
        setupStateEstimate();

        // Whole body control
        wbc_ = std::make_shared<HierarchicalWbc>(legged_interface_->getPinocchioInterface(),
                                                 legged_interface_->getCentroidalModelInfo(),
                                                 *eeKinematicsPtr_);
        // wbc_ = std::make_shared<WeightedWbc>(legged_interface_->getPinocchioInterface(),
        //                                      legged_interface_->getCentroidalModelInfo(),
        //                                      *eeKinematicsPtr_);
        wbc_->loadTasksSetting(task_file_, verbose_);

        // Safety Checker
        safety_checker_ = std::make_shared<SafetyChecker>(legged_interface_->getCentroidalModelInfo());

        // Terrain Estimator
        terrain_estimator_ = std::make_shared<TerrainEstimator>();

        // Stand Controller
        // 16-DOF 弹簧腿机器狗站立姿态
        // 关节顺序: [FL_hip, FL_thigh, FL_calf, FL_box,
        //            FR_hip, FR_thigh, FR_calf, FR_box,
        //            RL_hip, RL_thigh, RL_calf, RL_box,
        //            RR_hip, RR_thigh, RR_calf, RR_box]
        // 与 robot_control.yaml 中 joints 列表顺序对应
        const int num_joints = static_cast<int>(joint_names_.size());
        std::vector<double> middle_position(num_joints, 0.0);
        std::vector<double> final_position(num_joints, 0.0);

        if (num_joints >= 12) {
            // hip joints: 0, [3], [6], [9] or 0,1,2,3 depending on yaml order
            // Using robot_control.yaml order: FL_hip, FR_hip, RL_hip, RR_hip,
            // FL_thigh, FR_thigh, RL_thigh, RR_thigh, FL_calf, FR_calf, RL_calf, RR_calf
            // middle (crouching): thigh ~1.535, calf ~-2.486 for right legs; ~1.535, ~2.486 for left
            // final (standing): thigh ~0.72, calf ~-1.44 for right; ~0.72, ~1.44 for left
            // hip indices: 0,1,2,3 | thigh: 4,5,6,7 | calf: 8,9,10,11 | box: 12,13,14,15
            // Middle position
            middle_position[0] = 0.0;   // FL_hip
            middle_position[1] = 0.0;   // FR_hip
            middle_position[2] = 0.0;   // RL_hip
            middle_position[3] = 0.0;   // RR_hip
            middle_position[4] = 1.535; // FL_thigh
            middle_position[5] = 1.535; // FR_thigh
            middle_position[6] = -1.535;// RL_thigh
            middle_position[7] = -1.535;// RR_thigh
            middle_position[8] = -2.486;// FL_calf
            middle_position[9] = -2.486;// FR_calf
            middle_position[10] = 2.486;// RL_calf
            middle_position[11] = 2.486;// RR_calf
            // Final (standing) position
            final_position[0] = 0.0;    // FL_hip
            final_position[1] = 0.0;    // FR_hip
            final_position[2] = 0.0;    // RL_hip
            final_position[3] = 0.0;    // RR_hip
            final_position[4] = 0.72;   // FL_thigh
            final_position[5] = 0.72;   // FR_thigh
            final_position[6] = -0.72;  // RL_thigh
            final_position[7] = -0.72;  // RR_thigh
            final_position[8] = -1.44;  // FL_calf
            final_position[9] = -1.44;  // FR_calf
            final_position[10] = 1.44;  // RL_calf
            final_position[11] = 1.44;  // RR_calf
        }
        if (num_joints >= 16) {
            // box (prismatic spring) joints - natural rest position
            middle_position[12] = 0.12; // FL_box
            middle_position[13] = 0.12; // FR_box
            middle_position[14] = 0.12; // RL_box
            middle_position[15] = 0.12; // RR_box
            final_position[12] = 0.12;  // FL_box
            final_position[13] = 0.12;  // FR_box
            final_position[14] = 0.12;  // RL_box
            final_position[15] = 0.12;  // RR_box
        }
        stand_controller_ = std::make_shared<StandController>(middle_position, final_position);

        return CallbackReturn::SUCCESS;
    }

    Ocs2QuadrupedController::~Ocs2QuadrupedController()
    {
        controller_running_ = false;
        if (mpc_thread_.joinable())
        {
            mpc_thread_.join();
        }

        std::cerr << "########################################################################";
        std::cerr << "\n### MPC Benchmarking";
        std::cerr << "\n###   Maximum : " << mpc_timer_.getMaxIntervalInMilliseconds() << "[ms].";
        std::cerr << "\n###   Average : " << mpc_timer_.getAverageInMilliseconds() << "[ms]." << std::endl;
        std::cerr << "########################################################################";
        std::cerr << "\n### WBC Benchmarking";
        std::cerr << "\n###   Maximum : " << wbc_timer_.getMaxIntervalInMilliseconds() << "[ms].";
        std::cerr << "\n###   Average : " << wbc_timer_.getAverageInMilliseconds() << "[ms].";
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        std::cerr << "\n### Start subscribers!!!!!!!!!"<<std::endl;
        std::cerr << "\n### Start subscribers!!!!!!!!!"<<std::endl;
        std::cerr << "\n### Start subscribers!!!!!!!!!"<<std::endl;
        std::cerr << "\n### Start subscribers!!!!!!!!!"<<std::endl;
        std::cerr << "\n### Start subscribers!!!!!!!!!"<<std::endl;
        control_input_subscription_ = get_node()->create_subscription<custom_msgs::msg::UserCmds>(
            "/user_cmd", 10, [this](const custom_msgs::msg::UserCmds::SharedPtr msg)
            {
                std::cerr << "\n### Start subscribers user command!!!!!!!!!"<<std::endl;
                {
                    std::lock_guard<std::mutex> lock(ctrl_comp_.user_cmds_mutex_);
                    ctrl_comp_.user_cmds_.linear_x_input = msg->linear_x_input;
                    ctrl_comp_.user_cmds_.linear_y_input = msg->linear_y_input;
                    ctrl_comp_.user_cmds_.angular_y_input = msg->angular_y_input;
                    ctrl_comp_.user_cmds_.angular_z_input = msg->angular_z_input;
                    ctrl_comp_.user_cmds_.height_ratio = msg->height_ratio;
                    ctrl_comp_.user_cmds_.gait_name = msg->gait_name;
                    ctrl_comp_.user_cmds_.passive_enable = msg->passive_enable; 
                    RCLCPP_INFO(get_node()->get_logger(), "已处理 user_cmd: x=%.2f, y=%.2f, az=%.2f, gait=%s", 
                                msg->linear_x_input, msg->linear_y_input, msg->angular_z_input, msg->gait_name.c_str());
                }
            });

        sub_joy_ = get_node()->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(ctrl_comp_.user_cmds_mutex_);
                // Handle message
                ctrl_comp_.user_cmds_.height_ratio = 1.0;
                ctrl_comp_.user_cmds_.passive_enable = false;
                ctrl_comp_.user_cmds_.linear_x_input = msg->axes[1];
                ctrl_comp_.user_cmds_.linear_y_input = msg->axes[0];
                ctrl_comp_.user_cmds_.angular_z_input = msg->axes[2];
                if (msg->buttons[10]) // RB
                {
                    if (msg->buttons[3]) // Y
                    {
                        ctrl_comp_.user_cmds_.gait_name = "stance";
                    } else if (msg->buttons[0]) // A
                    {
                        ctrl_comp_.user_cmds_.gait_name = "bound";
                    } else if (msg->buttons[1]) // B
                    {
                        ctrl_comp_.user_cmds_.gait_name = "stance";
                    } else if (msg->buttons[2]) // X
                    {
                        ctrl_comp_.user_cmds_.gait_name = "trot";
                    }
                }
            });

        observation_publisher_ = get_node()->create_publisher<ocs2_msgs::msg::MpcObservation>(
            "legged_robot_mpc_observation", 10);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // clear out vectors in case of restart
        ctrl_comp_.clear();

        // assign command interfaces
        for (auto &interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // assign state interfaces
        for (auto &interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == imu_name_)
            {
                ctrl_comp_.imu_state_interface_.emplace_back(interface);
            }
            // else if (interface.get_prefix_name() == foot_force_name_)
            // {
            //     ctrl_comp_.foot_force_state_interface_.emplace_back(interface);
            // }
            else
            {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        if (mpc_running_ == false)
        {
            // Initial state
            ctrl_comp_.observation_.state.setZero(
                static_cast<long>(legged_interface_->getCentroidalModelInfo().stateDim));
            updateStateEstimation(modeNumber2StanceLeg(planned_mode), get_node()->now(), rclcpp::Duration(0, 1 / ctrl_comp_.frequency_ * 1000000000));
            ctrl_comp_.observation_.input.setZero(
                static_cast<long>(legged_interface_->getCentroidalModelInfo().inputDim));
            ctrl_comp_.observation_.mode = STANCE;

            const TargetTrajectories target_trajectories({ctrl_comp_.observation_.time},
                                                         {ctrl_comp_.observation_.state},
                                                         {ctrl_comp_.observation_.input});

            // Set the first observation and command and wait for optimization to finish
            mpc_mrt_interface_->setCurrentObservation(ctrl_comp_.observation_);
            mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(target_trajectories);
            RCLCPP_INFO(get_node()->get_logger(), "Waiting for the initial policy ...");
            while (!mpc_mrt_interface_->initialPolicyReceived())
            {
                mpc_mrt_interface_->advanceMpc();
                rclcpp::WallRate(legged_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
            }
            RCLCPP_INFO(get_node()->get_logger(), "Initial policy has been received.");

            mpc_running_ = true;
        }

        const int n_joints = static_cast<int>(joint_names_.size());
        std::vector<double> init_joint_pos(n_joints, 0.0);
        for (int i = 0; i < n_joints; i++)
        {
            init_joint_pos[i] = ctrl_comp_.joint_position_state_interface_[i].get().get_value();
        }
        stand_controller_->setInitPosition(init_joint_pos);

        RCLCPP_INFO(get_node()->get_logger(), "控制器已激活。当前状态: height_ratio=%.2f, gait=%s", 
                    ctrl_comp_.user_cmds_.height_ratio, ctrl_comp_.user_cmds_.gait_name.c_str());

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_shutdown(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_error(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    void Ocs2QuadrupedController::setupLeggedInterface()
    {
        legged_interface_ = std::make_shared<LeggedInterface>(task_file_, urdf_file_, reference_file_);
        legged_interface_->setupJointNames(joint_names_, feet_names_);

        LOG_TRACE((std::string("[DEBUG] urdf_file_: ") + urdf_file_).c_str());
        LOG_TRACE((std::string("[DEBUG] joint_names_.size() = ") + std::to_string(joint_names_.size())).c_str());
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            LOG_TRACE((std::string("  ") + std::to_string(i) + ": '" + joint_names_[i] + "'").c_str());
        }

        legged_interface_->setupOptimalControlProblem(task_file_, urdf_file_, reference_file_, verbose_);

        LOG_TRACE((std::string("[DEBUG] actuatedDofNum = ") + std::to_string(legged_interface_->getCentroidalModelInfo().actuatedDofNum)).c_str());
    }

    void Ocs2QuadrupedController::setupMpc()
    {
        mpc_ = std::make_shared<SqpMpc>(legged_interface_->mpcSettings(), legged_interface_->sqpSettings(),
                                        legged_interface_->getOptimalControlProblem(),
                                        legged_interface_->getInitializer());
        rbd_conversions_ = std::make_shared<CentroidalModelRbdConversions>(legged_interface_->getPinocchioInterface(),
                                                                           legged_interface_->getCentroidalModelInfo());

        // Initialize the reference manager
        const auto gait_manager_ptr = std::make_shared<GaitManager>(
            ctrl_comp_, legged_interface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule());
        gait_manager_ptr->init(gait_file_);
        mpc_->getSolverPtr()->addSynchronizedModule(gait_manager_ptr);
        mpc_->getSolverPtr()->setReferenceManager(legged_interface_->getReferenceManagerPtr());

        ctrl_comp_.target_manager_ = std::make_shared<TargetManager>(ctrl_comp_,
                                                                     legged_interface_->getReferenceManagerPtr(),
                                                                     task_file_, reference_file_,
                                                                     this->get_node());
    }

    void Ocs2QuadrupedController::setupMrt()
    {
        mpc_mrt_interface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
        mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());
        mpc_timer_.reset();

        controller_running_ = true;
        mpc_thread_ = std::thread([&]
                                  {
            while (controller_running_) {
                try {
                    executeAndSleep(
                        [&] {
                            if (mpc_running_) {
                                mpc_timer_.startTimer();
                                mpc_mrt_interface_->advanceMpc();
                                mpc_timer_.endTimer();
                            }
                        },
                        legged_interface_->mpcSettings().mpcDesiredFrequency_);
                } catch (const std::exception &e) {
                    controller_running_ = false;
                    RCLCPP_WARN(get_node()->get_logger(), "[Ocs2 MPC thread] Error : %s", e.what());
                }
            } });
        setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpc_thread_);
        RCLCPP_INFO(get_node()->get_logger(), "MRT initialized. MPC thread started.");
    }

    void Ocs2QuadrupedController::setupStateEstimate()
    {
        ctrl_comp_.estimator_ = std::make_shared<KalmanFilterEstimate>(legged_interface_->getPinocchioInterface(),
                                                                       legged_interface_->getCentroidalModelInfo(),
                                                                       *eeKinematicsPtr_, ctrl_comp_, this->get_node());
        dynamic_cast<KalmanFilterEstimate &>(*ctrl_comp_.estimator_).loadSettings(task_file_, verbose_);
        ctrl_comp_.observation_.time = 0;
    }

    void Ocs2QuadrupedController::updateStateEstimation(const contact_flag_t &contact_flag, const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        LOG_TRACE("[DEBUG] updateStateEstimation: entering");
        measured_rbd_state_ = ctrl_comp_.estimator_->update(contact_flag, time, period);
        LOG_TRACE("[DEBUG] updateStateEstimation: estimator->update done");
        ctrl_comp_.observation_.time += period.seconds();
        const scalar_t yaw_last = ctrl_comp_.observation_.state(9);
        LOG_TRACE("[DEBUG] updateStateEstimation: yaw_last loaded");
        ctrl_comp_.observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(measured_rbd_state_);
        LOG_TRACE("[DEBUG] updateStateEstimation: computeCentroidal done");
        ctrl_comp_.observation_.state(9) = yaw_last + angles::shortest_angular_distance(
                                                          yaw_last, ctrl_comp_.observation_.state(9));
        ctrl_comp_.observation_.mode = ctrl_comp_.estimator_->getMode();
        LOG_TRACE("[DEBUG] updateStateEstimation: getMode done");
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ocs2::legged_robot::Ocs2QuadrupedController, controller_interface::ControllerInterface);
