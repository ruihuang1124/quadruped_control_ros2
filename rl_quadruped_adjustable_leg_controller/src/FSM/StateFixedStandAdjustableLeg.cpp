//
// Created by lxq on 25-07-22.
//

#include "rl_quadruped_adjustable_leg_controller/FSM/StateFixedStandAdjustableLeg.h"
#include <vector>
#include <cmath>
#include <iostream>

StateFixedStandAdjustableLeg::StateFixedStandAdjustableLeg(CtrlInterfaces &ctrl_interfaces, const std::vector<double> &target_pos_adjustable_leg,
                                 const double kp,
                                 const double kd,
                                 const double kp_special,  // 新增：特殊关节的 kp
                                 const double kd_special  // 新增：特殊关节的 kd
                                 )
    : FSMState(FSMStateName::FIXEDSTANDADJUSTABLELEG, "fixed stand adjustable leg", ctrl_interfaces),
      kp_(kp), kd_(kd),
      kp_special_(kp_special), kd_special_(kd_special)
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
        // ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_);
        // ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_);

        if (i == 3 || i == 7 || i == 11 || i == 15) 
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

void StateFixedStandAdjustableLeg::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    percent_ += 1 / duration_;
    phase = std::tanh(percent_);

    // 更新时间计数器
    tick_count_++;
    double t = tick_count_ / ctrl_interfaces_.frequency_;

    // 获取控制周期 dt (例如 0.01s)
    double dt = 1.0 / ctrl_interfaces_.frequency_;
    double t_prev = t - dt; // 上一帧的时间

    // 定义正弦波参数
    const double center_pos = 0.05; // 中心位置 (也是开始和结束位置)
    const double amplitude = 0.015;  // 振幅 (0.12 - 0.09)
    const double omega = 1.0; 
    const double cycle_period = 2.0 * M_PI / omega; 
    const double max_duration = 3.0 * cycle_period; // 10个完整周期

    // 计算增量 (Delta)
    double delta = 0.0;

    // 更新 target_pos_adjustable_leg_ 的数值
    if (t < max_duration)
    {
        // 计算当前时刻的理论绝对位置
        double ideal_pos_now = center_pos + amplitude * sin(omega * t);
        
        // 计算上一时刻的理论绝对位置
        // 如果是第一帧(t_prev < 0)，则上一时刻位置就是中心点 0.09
        double ideal_pos_prev;
        if (t_prev < 0) {
            ideal_pos_prev = center_pos;
        } else {
            ideal_pos_prev = center_pos + amplitude * sin(omega * t_prev);
        }

        // 关键：只计算差值
        delta = ideal_pos_now - ideal_pos_prev;
        
        // 应用增量 (恢复平滑效果)
        target_pos_adjustable_leg_[3] += delta;
        target_pos_adjustable_leg_[7] += delta;
        target_pos_adjustable_leg_[11] += delta;
        target_pos_adjustable_leg_[15] += delta;
    }
    else
    {
        // 5. 结束处理
        // 理论上运行完整数个周期后，位置正好回到 0.09
        // 为了防止浮点数计算产生的微小漂移，这里强制锁定为 0.09
        // 因为此时 sin(omega * t) 应该非常接近 0，所以这里赋值不会跳变
        target_pos_adjustable_leg_[3] = center_pos;
        target_pos_adjustable_leg_[7] = center_pos;
        target_pos_adjustable_leg_[11] = center_pos;
        target_pos_adjustable_leg_[15] = center_pos;
    }
    for (int i = 0; i < 16; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(
            phase * target_pos_adjustable_leg_[i] + (1 - phase) * start_pos_adjustable_leg_[i]);
    }

    // // 更新时间计数器（假设100Hz控制频率）
    // tick_count_++;
    // double t = tick_count_ / ctrl_interfaces_.frequency_;
    // // std::cout << sin_amp_ * sin(t) << std::endl;

    
    // 更新特定关节的正弦变化值
    // target_pos_adjustable_leg_[8] += sin_amp_ * sin(t);
    // target_pos_adjustable_leg_[9] += sin_amp_ * sin(t);
    // target_pos_adjustable_leg_[10] += sin_amp_ * sin(t+1);
    // target_pos_adjustable_leg_[11] += sin_amp_ * sin(t+1);
    // target_pos_adjustable_leg_[12] += 100*sin_amp_ * sin(0.5*t);
    // target_pos_adjustable_leg_[13] += 100*sin_amp_ * sin(0.5*t);
    // target_pos_adjustable_leg_[14] += 100*sin_amp_ * sin(0.5*t+1);
    // target_pos_adjustable_leg_[15] += 100*sin_amp_ * sin(0.5*t+1);
    // target_pos_adjustable_leg_[3] += 100*sin_amp_ * sin(0.5*t);
    // target_pos_adjustable_leg_[7] += 100*sin_amp_ * sin(0.5*t);
    // target_pos_adjustable_leg_[11] += 100*sin_amp_ * sin(0.5*t);
    // target_pos_adjustable_leg_[15] += 100*sin_amp_ * sin(0.5*t);
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
