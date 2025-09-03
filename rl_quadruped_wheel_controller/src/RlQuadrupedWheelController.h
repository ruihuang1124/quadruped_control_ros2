//
// Created by tlab-uav on 24-10-4.
//

#ifndef LEGGEDGYMCONTROLLER_H
#define LEGGEDGYMCONTROLLER_H
#include <controller_interface/controller_interface.hpp>
// #include <rl_quadruped_wheel_controller/FSM/StateRL.h>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "rl_quadruped_wheel_controller/control/CtrlComponent.h"
// #include "controller_common/FSM/StateFixedDown.h"
// #include "rl_quadruped_wheel_controller/FSM/StateFixedStand.h"
// #include "controller_common//FSM/StatePassive.h"

#include "rl_quadruped_wheel_controller/FSM/StateQWFixedStand.h"
#include "rl_quadruped_wheel_controller/FSM/StateQWFixedDown.h"
#include "rl_quadruped_wheel_controller/FSM/StateQWPassive.h"
#include "rl_quadruped_wheel_controller/FSM/StateQWRL.h"
#include "custom_msgs/msg/ray_caster.hpp"

namespace rl_quadruped_wheel_controller
{
    struct FSMStateList
    {
        std::shared_ptr<FSMState> invalid;
        std::shared_ptr<StateQWPassive> passive;
        std::shared_ptr<StateQWFixedDown> fixedDown;
        std::shared_ptr<StateQWFixedStand> fixedStand;
        std::shared_ptr<StateQWRL> rl;
    };

    class LeggedWheelController final : public controller_interface::ControllerInterface
    {
    public:
        LeggedWheelController() = default;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State& previous_state) override;

    private:
        std::shared_ptr<FSMState> getNextState(FSMStateName stateName) const;

        CtrlComponent ctrl_component_;
        CtrlInterfaces ctrl_interfaces_;
        std::vector<std::string> joint_names_;
        std::string base_name_ = "base";
        std::vector<std::string> feet_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        std::string command_prefix_;

        // IMU Sensor
        std::string imu_name_;
        std::vector<std::string> imu_interface_types_;
        // Foot Force Sensor
        std::string foot_force_name_;
        std::vector<std::string> foot_force_interface_types_;

        // FR FL RR RL
        std::vector<double> stand_pos_ = {
            0.0, 0.7, -1.5, 0.0, // FL
            0.0, -0.7, 1.5, 0.0, // RL
            0.0, 0.7, -1.5, 0.0,// FR
            0.0, -0.7, 1.5, 0.0 // RR
        };

        std::vector<double> down_pos_ = {
            0.0, 1.27, -2.8, 0.0, // FL
            0.0, -1.27, 2.8, 0.0, // RL
            0.0, 1.27, -2.8, 0.0, // FR
            0.0, -1.27, 2.8, 0.0 // RR
        };

        double stand_kp_ = 100.0;
        double stand_kd_ = 3.5;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>*>
        command_interface_map_ = {
            {"effort", &ctrl_interfaces_.joint_torque_command_interface_},
            {"position", &ctrl_interfaces_.joint_position_command_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_},
            {"kp", &ctrl_interfaces_.joint_kp_command_interface_},
            {"kd", &ctrl_interfaces_.joint_kd_command_interface_}
        };

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>*>
        state_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_state_interface_},
            {"effort", &ctrl_interfaces_.joint_effort_state_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_}
        };

        rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
        rclcpp::Subscription<custom_msgs::msg::RayCaster>::SharedPtr quadruped_ray_caster_subscriber_;


        FSMMode mode_ = FSMMode::NORMAL;
        std::string state_name_;
        FSMStateName next_state_name_ = FSMStateName::INVALID;
        FSMStateList state_list_;
        std::shared_ptr<FSMState> current_state_;
        std::shared_ptr<FSMState> next_state_;
    };
}
#endif //LEGGEDGYMCONTROLLER_H
