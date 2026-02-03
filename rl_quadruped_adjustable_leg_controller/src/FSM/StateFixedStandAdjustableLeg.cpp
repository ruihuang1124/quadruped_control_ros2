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

    // // 更新时间计数器
    // tick_count_++;
    // double t = tick_count_ / ctrl_interfaces_.frequency_;

    // // 获取控制周期 dt (例如 0.01s)
    // double dt = 1.0 / ctrl_interfaces_.frequency_;
    // double t_prev = t - dt; // 上一帧的时间

    // // 定义正弦波参数
    // const double center_pos = 0.05; // 中心位置 (也是开始和结束位置)
    // const double target_amplitude = 0.015;  // 振幅 (0.12 - 0.09)
    // const double omega = 1.0; 
    // const double cycle_period = 2.0 * M_PI / omega; 
    // const double max_duration = 3.0 * cycle_period; // 10个完整周期

    // // 软启动时间：在前 2.0 秒内，振幅从 0 变到 0.03
    // // 这保证了所有相位的腿都从静止(0.09)平滑开始
    // const double ramp_time = 2.0; 

    // // 定义四条腿的索引和对应的相位 (0, 90, 180, 270度)
    // // 这样四条腿就像波浪一样依次运动
    // struct LegConfig {
    //     int index;
    //     double phase_offset;
    // };
    // std::vector<LegConfig> legs = {
    //     {3,  0.0},           // 0度
    //     {7,  M_PI / 2.0},    // 90度
    //     {11, M_PI},          // 180度
    //     {15, 3.0 * M_PI / 2.0} // 270度
    // };

    // if (t < max_duration)
    // {
    //     // 计算当前的振幅系数 (0.0 ~ 1.0)
    //     // 如果 t > ramp_time，系数固定为 1.0
    //     double ramp_now = (t < ramp_time) ? (t / ramp_time) : 1.0;
        
    //     // 计算上一帧的振幅系数
    //     double ramp_prev = 0.0;
    //     if (t_prev > 0) {
    //         ramp_prev = (t_prev < ramp_time) ? (t_prev / ramp_time) : 1.0;
    //     }

    //     // 遍历四条腿，分别计算各自的 Delta
    //     for (const auto& leg : legs)
    //     {
    //         // --- 计算当前时刻理论位置 ---
    //         // 公式：中心 + (动态振幅) * sin(wt + phi)
    //         double ideal_now = center_pos + (target_amplitude * ramp_now) * sin(omega * t + leg.phase_offset);

    //         // --- 计算上一时刻理论位置 ---
    //         double ideal_prev = center_pos; // 默认为中心
    //         if (t_prev > 0) {
    //             ideal_prev = center_pos + (target_amplitude * ramp_prev) * sin(omega * t_prev + leg.phase_offset);
    //         }

    //         // --- 计算差值并应用 ---
    //         double delta = ideal_now - ideal_prev;
    //         target_pos_adjustable_leg_[leg.index] += delta;
    //     }
    // }
    // else
    // {
    //     // 结束时全部归位
    //     target_pos_adjustable_leg_[3] = center_pos;
    //     target_pos_adjustable_leg_[7] = center_pos;
    //     target_pos_adjustable_leg_[11] = center_pos;
    //     target_pos_adjustable_leg_[15] = center_pos;
    // }
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
