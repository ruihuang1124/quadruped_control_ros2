//
// Created by tlab-uav on 24-10-4.
//

#include "RlQuadrupedControllerAdjustableLeg.h"
#include <iostream>

namespace rl_quadruped_controller_adjustable_leg
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
                // Handle message
                ctrl_interfaces_.control_inputs_.command = msg->command;
                ctrl_interfaces_.control_inputs_.lx = msg->lx;
                ctrl_interfaces_.control_inputs_.ly = msg->ly;
                ctrl_interfaces_.control_inputs_.rx = msg->rx;
                ctrl_interfaces_.control_inputs_.ry = msg->ry;
            });

        // pose_control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::PoseCmdInputs>(
        //     "/pose_control_input", 10, [this](const control_input_msgs::msg::PoseCmdInputs::SharedPtr msg)
        //     {
        //         // Handle message
        //         ctrl_interfaces_.pose_cmd_inputs_.pos_x = msg->pos_x;
        //         ctrl_interfaces_.pose_cmd_inputs_.pos_y = msg->pos_y;
        //         ctrl_interfaces_.pose_cmd_inputs_.pos_z = msg->pos_z;
        //         ctrl_interfaces_.pose_cmd_inputs_.pos_roll = msg->pos_roll;
        //         ctrl_interfaces_.pose_cmd_inputs_.pos_pitch = msg->pos_pitch;
        //         ctrl_interfaces_.pose_cmd_inputs_.pos_yaw = msg->pos_yaw;
        //         ctrl_interfaces_.pose_cmd_inputs_.pos_quat_x = msg->pos_quat_x;
        //         ctrl_interfaces_.pose_cmd_inputs_.pos_quat_y = msg->pos_quat_y;
        //         ctrl_interfaces_.pose_cmd_inputs_.pos_quat_z = msg->pos_quat_z;
        //         ctrl_interfaces_.pose_cmd_inputs_.pos_quat_w = msg->pos_quat_w;
        //     });

        sub_joy_ = get_node()->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg)
            {
                // Handle message
                ctrl_interfaces_.control_inputs_.lx = msg->axes[1];
                ctrl_interfaces_.control_inputs_.ly = msg->axes[0];
                ctrl_interfaces_.control_inputs_.rx = -msg->axes[2];
                if (msg->buttons[10]) // RB
                {
                    if (msg->buttons[3]) // Y
                    {
                        ctrl_interfaces_.control_inputs_.command = 2;
                    }
                    else if (msg->buttons[0]) // A
                    {
                        ctrl_interfaces_.control_inputs_.command = 1;
                    } else if (msg->buttons[1]) // B
                    {
                        ctrl_interfaces_.control_inputs_.command = 0;
                    } else if (msg->buttons[2]) // X
                    {
                        ctrl_interfaces_.control_inputs_.command = 3;
                    }
                }
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
        state_list_.fixedDownAdjustableLeg = std::make_shared<StateFixedDownAdjustableLeg>(ctrl_interfaces_, down_pos_, stand_kp_, stand_kd_);
        // state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_interfaces_, stand_pos_, stand_kp_, stand_kd_);
        state_list_.fixedStandAdjustableLeg = std::make_shared<StateFixedStandAdjustableLeg>(ctrl_interfaces_, stand_pos_, stand_kp_, stand_kd_, stand_pos_adjustable_leg_);
        state_list_.rl = std::make_shared<StateRL>(ctrl_interfaces_, ctrl_component_, stand_pos_adjustable_leg_);

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
        default:
            return state_list_.invalid;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rl_quadruped_controller_adjustable_leg::LeggedGymController, controller_interface::ControllerInterface);
// PLUGINLIB_EXPORT_CLASS(rl_quadruped_controller::LeggedGymController, controller_interface::ControllerInterface);
