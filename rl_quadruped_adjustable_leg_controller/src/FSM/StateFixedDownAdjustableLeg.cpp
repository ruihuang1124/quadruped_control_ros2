//
// Created by tlab-uav on 24-9-11.
//

#include "rl_quadruped_adjustable_leg_controller/FSM/StateFixedDownAdjustableLeg.h"

#include <cmath>

StateFixedDownAdjustableLeg::StateFixedDownAdjustableLeg(CtrlInterfaces& ctrl_interfaces,
                               const std::vector<double>& target_pos,
                                 const double kp,
                                 const double kd,
                                 const double kp_special,  // 新增：特殊关节的 kp
                                 const double kd_special  // 新增：特殊关节的 kd
                                 )
    : FSMState(FSMStateName::FIXEDDOWNADJUSTABLELEG, "fixed down adjustable leg", ctrl_interfaces),
      kp_(kp), kd_(kd),
      kp_special_(kp_special), kd_special_(kd_special)
{
    duration_ = ctrl_interfaces_.frequency_ * 1.2;
    for (int i = 0; i < 16; i++)
    {
        target_pos_[i] = target_pos[i];
    }
}

void StateFixedDownAdjustableLeg::enter()
{
    for (int i = 0; i < 16; i++)
    {
        start_pos_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
    }
    for (int i = 0; i < 16; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(start_pos_[i]);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0);
        // ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_);
        // ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_);

        if (i == 12 || i == 13 || i == 14 || i == 15) 
        {
            // 对特殊关节使用特殊的 kp 和 kd
            ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_special_);
            ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_special_);
        }
        else 
        {
            // 对其他普通关节使用通用的 kp 和 kd
            ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_);
            ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_);
        }

    }
    ctrl_interfaces_.control_inputs_.command = -1;
}

void StateFixedDownAdjustableLeg::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    percent_ += 1 / duration_;
    phase = std::tanh(percent_);
    for (int i = 0; i < 16; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(
            phase * target_pos_[i] + (1 - phase) * start_pos_[i]);
    }
}

void StateFixedDownAdjustableLeg::exit()
{
    percent_ = 0;
}

FSMStateName StateFixedDownAdjustableLeg::checkChange()
{
    if (percent_ < 1.5) {
        return FSMStateName::FIXEDDOWNADJUSTABLELEG;
    }
    switch (ctrl_interfaces_.control_inputs_.command) {
    case -1:
        return FSMStateName::FIXEDDOWNADJUSTABLELEG;
    case 0:
        return FSMStateName::PASSIVEADJUSTABLELEG;
    case 2:
        return FSMStateName::FIXEDSTANDADJUSTABLELEG;
    default:
        return FSMStateName::FIXEDDOWNADJUSTABLELEG;
    }
}
