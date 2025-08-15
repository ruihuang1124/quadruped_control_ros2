//
// Created by ray on 2025-07-25.
//

#include "rl_quadruped_wheel_controller/FSM/StateQWFixedStand.h"

#include <cmath>
#include <iostream>

StateQWFixedStand::StateQWFixedStand(CtrlInterfaces& ctrl_interfaces, const std::vector<double>& target_pos,
                               const double kp,
                               const double kd)
    : FSMState(FSMStateName::QWFIXEDSTAND, "qw fixed stand", ctrl_interfaces),
      kp_(kp), kd_(kd)
{
    duration_ = ctrl_interfaces_.frequency_ * 1.2;
    for (int i = 0; i < num_joints_; i++)
    {
        target_pos_[i] = target_pos[i];
    }
}

void StateQWFixedStand::enter()
{
    std::cout <<"start entering fixed stand fsm!!! "<< std::endl;
    for (int i = 0; i < num_joints_; i++)
    {
        start_pos_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
    }
    for (int i = 0; i < num_joints_; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(start_pos_[i]);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_);
    }
    ctrl_interfaces_.control_inputs_.command = 0;
    std::cout <<"finish entering fixed stand fsm!!! "<< std::endl;
}

void StateQWFixedStand::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    percent_ += 1 / duration_;
    phase = std::tanh(percent_);
    for (int i = 0; i < num_joints_; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(
            phase * target_pos_[i] + (1 - phase) * start_pos_[i]);
    }
}

void StateQWFixedStand::exit()
{
    percent_ = 0;
}

FSMStateName StateQWFixedStand::checkChange()
{
    if (percent_ < 1.5)
    {
        return FSMStateName::QWFIXEDSTAND;
    }
    switch (ctrl_interfaces_.control_inputs_.command) {
    case 0:
        return FSMStateName::QWPASSIVE;
    case 1:
        return FSMStateName::QWFIXEDDOWN;
    case 3:
        return FSMStateName::QWRL;
    default:
        return FSMStateName::QWFIXEDSTAND;
    }
}
