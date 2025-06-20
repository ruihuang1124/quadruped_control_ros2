//
// Created by tlab-uav on 24-9-24.
//

#ifndef JOINTPOSITIONCONTROLLER_H
#define JOINTPOSITIONCONTROLLER_H

#include <controller_interface/controller_interface.hpp>
#include "controller_common/CtrlInterfaces.h"

namespace piper_example_controllers {

    class JointPositionController final : public controller_interface::ControllerInterface {
    public:
        JointPositionController() = default;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;


        controller_interface::CallbackReturn on_init() override;


        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

    protected:


        std::string state_name_;
        std::array<double, 7> initial_q_{0, 0, 0, 0, 0, 0, 0};
        bool initialization_flag_{true};
        bool norminal_position_flag_{false};
        const int num_joints = 7;
        double elapsed_time_ = 0.0;
        double norminal_progress_=0.0;
        double trajectory_period_ = 0.001;
        double moving_norminal_pisition_duration = 5.0;

        CtrlInterfaces ctrl_interfaces_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;
        double joint_position_norminal_[7] = {0.0, 1.5, -1.5, 0.0, 0.122, 0.0, 0.024};

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> > *>
        command_interface_map_ = {
            {"effort", &ctrl_interfaces_.joint_torque_command_interface_},
            {"position", &ctrl_interfaces_.joint_position_command_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_},
            {"kp", &ctrl_interfaces_.joint_kp_command_interface_},
            {"kd", &ctrl_interfaces_.joint_kd_command_interface_}
        };
        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > *>
        state_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_state_interface_},
            {"effort", &ctrl_interfaces_.joint_effort_state_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_}
        };

        std::string command_prefix_;
    };
}

#endif //JOINTPOSITIONCONTROLLER_H
