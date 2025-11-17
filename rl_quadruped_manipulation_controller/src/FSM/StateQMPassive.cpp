//
// Created by ray on 2025-07-25.
//

#include "rl_quadruped_manipulation_controller/FSM/StateQMPassive.h"

#include <iostream>

StateQMPassive::StateQMPassive(CtrlInterfaces& ctrl_interfaces) : FSMState(
    FSMStateName::QMPASSIVE, "qm passive", ctrl_interfaces)
{
}

void StateQMPassive::enter()
{
    for (auto i : ctrl_interfaces_.joint_torque_command_interface_)
    {
        i.get().set_value(0);
    }
    for (auto i : ctrl_interfaces_.joint_position_command_interface_)
    {
        i.get().set_value(0);
    }
    for (auto i : ctrl_interfaces_.joint_velocity_command_interface_)
    {
        i.get().set_value(0);
    }
    for (auto i : ctrl_interfaces_.joint_kp_command_interface_)
    {
        i.get().set_value(0);
    }
    for (auto i : ctrl_interfaces_.joint_kd_command_interface_)
    {
        i.get().set_value(1);
    }
    ctrl_interfaces_.control_inputs_.command = 0;
}

void StateQMPassive::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    // for (auto i : ctrl_interfaces_.joint_position_state_interface_)
    // {
    //     std::cout<< i.get().get_prefix_name()<<std::endl;
    //     std::cout<< i.get().get_value()<<std::endl;
    // }
}

void StateQMPassive::exit()
{
}

FSMStateName StateQMPassive::checkChange()
{
    if (ctrl_interfaces_.control_inputs_.command == 1)
    {
        return FSMStateName::QMFIXEDDOWN;
    }
    return FSMStateName::QMPASSIVE;
}
