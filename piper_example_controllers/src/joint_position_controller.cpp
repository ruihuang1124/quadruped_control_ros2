//
// Created by tlab-uav on 24-9-24.
//

#include <iterator>

#include "piper_example_controllers/joint_pisition_controller.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <angles/angles.h>

namespace piper_example_controllers
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration JointPositionController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

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

    controller_interface::InterfaceConfiguration JointPositionController::state_interface_configuration() const
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

        return conf;
    }

    controller_interface::return_type JointPositionController::update(const rclcpp::Time& time,
                                                                      const rclcpp::Duration& period)
    {
        // std::cerr<<"here update!!!!!!!!!"<<std::endl;
        // for (int i = 0; i < 7; i++)
        // {
        //     joint_states_current_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        //     std::cerr<<"current joint states for joint " << i<<" is: "<<joint_states_current_[i]<<std::endl;
        // }

        if (initialization_flag_)
        {
            for (int i = 0; i < num_joints; ++i)
            {
                initial_q_.at(i) = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
            }
            initialization_flag_ = false;
            norminal_progress_ = 0.0;
            elapsed_time_ = 0.0;
            for (int i = 0; i < num_joints; ++i)
            {
                ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(initial_q_.at(i));
            }
        }
        else if (!norminal_position_flag_)
        {
            norminal_progress_ += trajectory_period_;

            for (int i = 0; i < num_joints; ++i)
            {
                double delta_angle = norminal_progress_/moving_norminal_pisition_duration * (joint_position_norminal_[i] - initial_q_.at(i));
                // std::cerr<<"desired joint angle for joint " << i<<" is: "<<initial_q_.at(i) + delta_angle<<std::endl;
                ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(initial_q_.at(i) + delta_angle);
                // ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(initial_q_.at(i) + delta_angle);
            }
            if (norminal_progress_ >= moving_norminal_pisition_duration)
            {
                for (int i = 0; i < num_joints; ++i)
                {
                    initial_q_.at(i) = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
                }
                norminal_progress_ = 0.0;
                norminal_position_flag_ = true;
            }

        }
        else {
            elapsed_time_ += trajectory_period_;
            double delta_angle = M_PI / 8 * (1 - std::cos(M_PI / 5.0 * elapsed_time_)) * 0.2;

            for (int i = 0; i < num_joints; ++i)
            {
                // std::cerr<<"desired joint angle for joint " << i<<" is: "<<initial_q_.at(i) + delta_angle<<std::endl;
                ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(initial_q_.at(i) + delta_angle);
            }
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn JointPositionController::on_init()
    {
        get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
        RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

        // Hardware Parameters
        command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ =
            auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        state_interface_types_ =
            auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn JointPositionController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn JointPositionController::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        initialization_flag_ = true;
        norminal_position_flag_ = false;
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
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn JointPositionController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn JointPositionController::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn JointPositionController::on_shutdown(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn JointPositionController::on_error(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(piper_example_controllers::JointPositionController, controller_interface::ControllerInterface);
