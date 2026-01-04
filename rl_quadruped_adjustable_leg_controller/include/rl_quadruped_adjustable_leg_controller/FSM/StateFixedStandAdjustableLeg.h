//
// Created by lxq on 24-9-10.
//

#ifndef STATEFIXEDSTANDADJUSTABLELEG_H
#define STATEFIXEDSTANDADJUSTABLELEG_H

#include <vector>
#include "controller_common/FSM/FSMState.h"

class StateFixedStandAdjustableLeg : public FSMState {
public:
    explicit StateFixedStandAdjustableLeg(CtrlInterfaces &ctrl_interfaces,
                             const std::vector<double> &target_pos,
                             double kp,
                             double kd,
                             const std::vector<double> &target_pos_adjustable_leg);
    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;
private:
    std::array<double, 16> target_pos_adjustable_leg_;
    std::array<double, 16> start_pos_adjustable_leg_; 
    rclcpp::Time start_time_;

    double kp_, kd_;

    double duration_ = 600; // steps
    double percent_ = 0; //%
    double phase = 0.0;

    int tick_count_ = 0;
    double sin_freq_ = 1.0; // 正弦波频率(Hz)
    double sin_amp_ = 0.001;  // 振幅(m/rad)
};


#endif //STATEFIXEDSTANDADJUSTABLELEG_H
