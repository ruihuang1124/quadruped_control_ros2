//
// Created by tlab-uav on 24-10-4.
//

#include "RlQuadrupedControllerAdjustableLeg.h"
#include <iostream>

namespace rl_quadruped_adjustable_leg_controller
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration LeggedGymController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
        
        std::cout << "222222222222222222222222" << std::endl;
        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" += interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" += interface_type);
                }
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration LeggedGymController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        for (const auto& interface_type : imu_interface_types_)
        {
            conf.names.push_back(imu_name_ + "/" += interface_type);
        }

        // for (const auto& interface_type : foot_force_interface_types_)
        // {
        //     conf.names.push_back(foot_force_name_ + "/" += interface_type);
        // }

        return conf;
    }

    controller_interface::return_type LeggedGymController::
    update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        applyLatestControlInputSnapshot();

        if (ctrl_component_.enable_estimator_)
        {
            if (ctrl_component_.robot_model_ == nullptr)
            {
                return controller_interface::return_type::OK;
            }

            ctrl_component_.robot_model_->update();
            ctrl_component_.estimator_->update();
        }

        if (mode_ == FSMMode::NORMAL)
        {
            current_state_->run(time, period);
            next_state_name_ = current_state_->checkChange();
            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            current_state_->exit();
            current_state_ = next_state_;

            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }

        return controller_interface::return_type::OK;
    }

    void LeggedGymController::applyLatestControlInputSnapshot()
    {
        const ControlInputSnapshot* const snapshot = control_input_buffer_.readFromRT();
        if (snapshot == nullptr || snapshot->sequence == last_applied_control_input_sequence_)
        {
            return;
        }

        // Axis fields are never written by FSM states, so every new complete
        // callback snapshot may replace them as one coherent set.
        ctrl_interfaces_.control_inputs_.lx = snapshot->inputs.lx;
        ctrl_interfaces_.control_inputs_.ly = snapshot->inputs.ly;
        ctrl_interfaces_.control_inputs_.rx = snapshot->inputs.rx;
        ctrl_interfaces_.control_inputs_.ry = snapshot->inputs.ry;

        // FSM enter() methods consume commands by writing a neutral value into
        // ctrl_interfaces_.control_inputs_. A Joy axis-only message must not
        // resurrect the previously consumed command, so command_sequence only
        // advances when a callback explicitly supplies a new command.
        if (snapshot->command_sequence != last_applied_control_command_sequence_)
        {
            ctrl_interfaces_.control_inputs_.command = snapshot->inputs.command;
            last_applied_control_command_sequence_ = snapshot->command_sequence;
        }
        last_applied_control_input_sequence_ = snapshot->sequence;
    }

    controller_interface::CallbackReturn LeggedGymController::on_init()
    {
        try
        {
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            feet_names_ = auto_declare<std::vector<std::string>>("feet_names", feet_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            base_name_ = auto_declare<std::string>("base_name", base_name_);

            // imu sensor
            imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
            imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", state_interface_types_);

            // foot_force_sensor
            // foot_force_name_ = auto_declare<std::string>("foot_force_name", foot_force_name_);
            // foot_force_interface_types_ =
            //     auto_declare<std::vector<std::string>>("foot_force_interfaces", foot_force_interface_types_);
            feet_force_threshold_ = auto_declare<double>("feet_force_threshold", feet_force_threshold_);

            // pose parameters
            down_pos_ = auto_declare<std::vector<double>>("down_pos", down_pos_);
            stand_pos_ = auto_declare<std::vector<double>>("stand_pos", stand_pos_);
            stand_kp_ = auto_declare<double>("stand_kp", stand_kp_);
            stand_kd_ = auto_declare<double>("stand_kd", stand_kd_);
            stand_pos_adjustable_leg_ = auto_declare<std::vector<double>>("stand_pos_adjustable_leg", stand_pos_adjustable_leg_);

            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Update Rate: %d Hz", ctrl_interfaces_.frequency_);

            if (foot_force_interface_types_.size() == 4)
            {
                RCLCPP_INFO(get_node()->get_logger(), "Enable Estimator");
                ctrl_component_.enable_estimator_ = true;
                ctrl_component_.estimator_ = std::make_shared<Estimator>(ctrl_interfaces_, ctrl_component_);
            }
            ctrl_component_.node_ = get_node();
            // ctrl_interfaces_.pose_cmd_inputs_.pos_x = 0.55;
            // ctrl_interfaces_.pose_cmd_inputs_.pos_y = 0.0;
            // ctrl_interfaces_.pose_cmd_inputs_.pos_z = 0.30;
            // ctrl_interfaces_.pose_cmd_inputs_.pos_roll = 0.0;
            // ctrl_interfaces_.pose_cmd_inputs_.pos_pitch = 3.14;
            // ctrl_interfaces_.pose_cmd_inputs_.pos_yaw = 0.0;
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LeggedGymController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                if (ctrl_component_.enable_estimator_)
                {
                    ctrl_component_.robot_model_ = std::make_shared<QuadrupedRobot>(
                        ctrl_interfaces_, msg->data, feet_names_, base_name_);
                }
            });


        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(control_input_writer_mutex_);
                pending_control_input_.inputs = *msg;
                ++pending_control_input_.sequence;
                ++pending_control_input_.command_sequence;
                control_input_buffer_.writeFromNonRT(pending_control_input_);
            });

        sub_joy_ = get_node()->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(control_input_writer_mutex_);
                pending_control_input_.inputs.lx = 0.5*msg->axes[1];
                pending_control_input_.inputs.ly = 0.34*msg->axes[0];
                pending_control_input_.inputs.rx = -1*msg->axes[2];
                bool command_updated = false;
                if (msg->buttons[10]) // RB
                {
                    if (msg->buttons[9] && msg->buttons[3]) // RB + LB + Y
                    {
                        pending_control_input_.inputs.command = 4;
                        command_updated = true;
                    }
                    else if (msg->buttons[3]) // Y
                    {
                        pending_control_input_.inputs.command = 2;
                        command_updated = true;
                    }
                    else if (msg->buttons[0]) // A
                    {
                        pending_control_input_.inputs.command = 1;
                        command_updated = true;
                    } else if (msg->buttons[1]) // B
                    {
                        pending_control_input_.inputs.command = 0;
                        command_updated = true;
                    } else if (msg->buttons[2]) // X
                    {
                        pending_control_input_.inputs.command = 3;
                        command_updated = true;
                    }
                }

                ++pending_control_input_.sequence;
                if (command_updated)
                {
                    ++pending_control_input_.command_sequence;
                }
                control_input_buffer_.writeFromNonRT(pending_control_input_);

                // // Handle message for xbox wireless controller
                // ctrl_interfaces_.control_inputs_.lx = 0.5*msg->axes[1];
                // ctrl_interfaces_.control_inputs_.ly = 0.5*msg->axes[0];
                // ctrl_interfaces_.control_inputs_.rx = msg->axes[2];
                // if (msg->buttons[6]) // RB
                // {
                //     if (msg->buttons[9]) // Y
                //     {
                //         ctrl_interfaces_.control_inputs_.command = 2;
                //     }
                //     else if (msg->buttons[0]) // A
                //     {
                //         ctrl_interfaces_.control_inputs_.command = 1;
                //     } else if (msg->buttons[1]) // B
                //     {
                //         ctrl_interfaces_.control_inputs_.command = 0;
                //     } else if (msg->buttons[3]) // X
                //     {
                //         ctrl_interfaces_.control_inputs_.command = 3;
                //     }
                // } //for xbox wireless controller
            });

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LeggedGymController::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();

        // assign command interfaces
        for (auto& interface : command_interfaces_)
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
        for (auto& interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == imu_name_)
            {
                ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
            }
            // else if (interface.get_prefix_name() == foot_force_name_)
            // {
            //     ctrl_interfaces_.foot_force_state_interface_.emplace_back(interface);
            // }
            else
            {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        // Create FSM List
        // state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
        state_list_.passiveAdjustableLeg = std::make_shared<StatePassiveAdjustableLeg>(ctrl_interfaces_);
        // state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_interfaces_, down_pos_, stand_kp_, stand_kd_);
        state_list_.fixedDownAdjustableLeg = std::make_shared<StateFixedDownAdjustableLeg>(ctrl_interfaces_, down_pos_, stand_kp_, stand_kd_, stand_kp_prismatic_joint, stand_kd_prismatic_joint);
        // state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_interfaces_, stand_pos_, stand_kp_, stand_kd_);
        state_list_.fixedStandAdjustableLeg = std::make_shared<StateFixedStandAdjustableLeg>(ctrl_interfaces_, stand_pos_adjustable_leg_, stand_kp_, stand_kd_, stand_kp_prismatic_joint, stand_kd_prismatic_joint);
        state_list_.rl = std::make_shared<StateRL>(
            ctrl_interfaces_, ctrl_component_, stand_pos_adjustable_leg_, joint_names_);
        state_list_.rlPolicy2 = std::make_shared<StateRL>(
            ctrl_interfaces_,
            ctrl_component_,
            stand_pos_adjustable_leg_,
            joint_names_,
            FSMStateName::RLPOLICY2,
            "rl policy2",
            "model_folder_policy2",
            "model_name_policy2");

        // Initialize FSM
        current_state_ = state_list_.passiveAdjustableLeg;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LeggedGymController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        if (current_state_)
        {
            current_state_->exit();
        }
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    LeggedGymController::on_cleanup(const rclcpp_lifecycle::State& previous_state)
    {
        return ControllerInterface::on_cleanup(previous_state);
    }

    controller_interface::CallbackReturn
    LeggedGymController::on_shutdown(const rclcpp_lifecycle::State& previous_state)
    {
        return ControllerInterface::on_shutdown(previous_state);
    }

    controller_interface::CallbackReturn LeggedGymController::on_error(const rclcpp_lifecycle::State& previous_state)
    {
        return ControllerInterface::on_error(previous_state);
    }

    std::shared_ptr<FSMState> LeggedGymController::getNextState(const FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::INVALID:
            return state_list_.invalid;
        // case FSMStateName::PASSIVE:
        //     return state_list_.passive;
        case FSMStateName::PASSIVEADJUSTABLELEG:
            return state_list_.passiveAdjustableLeg;
        // case FSMStateName::FIXEDDOWN:
        //     return state_list_.fixedDown;
        case FSMStateName::FIXEDDOWNADJUSTABLELEG:
            return state_list_.fixedDownAdjustableLeg;
        // case FSMStateName::FIXEDSTAND:
        //     return state_list_.fixedStand;
        case FSMStateName::FIXEDSTANDADJUSTABLELEG:
            return state_list_.fixedStandAdjustableLeg;
        case FSMStateName::RL:
            return state_list_.rl;
        case FSMStateName::RLPOLICY2:
            return state_list_.rlPolicy2;
        default:
            return state_list_.invalid;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rl_quadruped_adjustable_leg_controller::LeggedGymController, controller_interface::ControllerInterface);
// PLUGINLIB_EXPORT_CLASS(rl_quadruped_controller::LeggedGymController, controller_interface::ControllerInterface);
