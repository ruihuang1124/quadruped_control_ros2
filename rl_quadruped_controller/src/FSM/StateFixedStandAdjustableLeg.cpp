//
// Created by lxq on 25-07-22.
//

#include "rl_quadruped_controller/FSM/StateFixedStandAdjustableLeg.h"
#include <vector>
#include <cmath>
#include <iostream>

StateFixedStandAdjustableLeg::StateFixedStandAdjustableLeg(CtrlInterfaces &ctrl_interfaces, const std::vector<double> &target_pos,
                                 const double kp,
                                 const double kd,
                                 const std::vector<double> &target_pos_adjustable_leg)
    : FSMState(FSMStateName::FIXEDSTANDADJUSTABLELEG, "fixed stand adjustable leg", ctrl_interfaces),
      kp_(kp), kd_(kd)
{
    duration_ = ctrl_interfaces_.frequency_ * 1.2;
    for (int i = 0; i < 16; i++)
    {
        target_pos_adjustable_leg_[i] = target_pos_adjustable_leg[i];
    }
}

void StateFixedStandAdjustableLeg::enter()
{
    for (int i = 0; i < 16; i++)
    {
        start_pos_adjustable_leg_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
    }
    for (int i = 0; i < 16; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(start_pos_adjustable_leg_[i]);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_);
    }
    ctrl_interfaces_.control_inputs_.command = -1;
}

void StateFixedStandAdjustableLeg::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    percent_ += 1 / duration_;
    phase = std::tanh(percent_);
    for (int i = 0; i < 16; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(
            phase * target_pos_adjustable_leg_[i] + (1 - phase) * start_pos_adjustable_leg_[i]);
    }

    // // 更新时间计数器（假设100Hz控制频率）
    // tick_count_++;
    // double t = tick_count_ / ctrl_interfaces_.frequency_;
    // // std::cout << sin_amp_ * sin(t) << std::endl;

    
    // // 更新特定关节的正弦变化值
    // target_pos_adjustable_leg_[8] += sin_amp_ * sin(t);
    // target_pos_adjustable_leg_[9] += sin_amp_ * sin(t);
    // target_pos_adjustable_leg_[10] += sin_amp_ * sin(t+1);
    // target_pos_adjustable_leg_[11] += sin_amp_ * sin(t+1);
    // target_pos_adjustable_leg_[12] += 100*sin_amp_ * sin(t);
    // target_pos_adjustable_leg_[13] += 100*sin_amp_ * sin(t);
    // target_pos_adjustable_leg_[14] += 100*sin_amp_ * sin(t+1);
    // target_pos_adjustable_leg_[15] += 100*sin_amp_ * sin(t+1);
}

void StateFixedStandAdjustableLeg::exit()
{
    percent_ = 0;
}

FSMStateName StateFixedStandAdjustableLeg::checkChange() {
    if (percent_ < 1.5) {
        return FSMStateName::FIXEDSTANDADJUSTABLELEG;
    }
    switch (ctrl_interfaces_.control_inputs_.command) {
        case -1:
            return FSMStateName::FIXEDSTANDADJUSTABLELEG;
        case 0:
            return FSMStateName::PASSIVEADJUSTABLELEG;
        case 1:
            return FSMStateName::FIXEDDOWNADJUSTABLELEG;
        case 3:
            return FSMStateName::RL;
        default:
            return FSMStateName::FIXEDSTANDADJUSTABLELEG;
    }
}
