//
// Created by tlab-uav on 25-2-27.
//

#include "rl_quadruped_controller/FSM/StatePassiveAdjustableLeg.h"

#include <iostream>

StatePassiveAdjustableLeg::StatePassiveAdjustableLeg(CtrlInterfaces& ctrl_interfaces) : FSMState(
    FSMStateName::PASSIVEADJUSTABLELEG, "passive adjustable leg", ctrl_interfaces)
{
}

void StatePassiveAdjustableLeg::enter()
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

void StatePassiveAdjustableLeg::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
}

void StatePassiveAdjustableLeg::exit()
{
}

FSMStateName StatePassiveAdjustableLeg::checkChange()
{
    if (ctrl_interfaces_.control_inputs_.command == 1)
    {
        return FSMStateName::FIXEDDOWNADJUSTABLELEG;
        // return FSMStateName::FIXEDDOWN;
    }
    return FSMStateName::PASSIVEADJUSTABLELEG;
}
