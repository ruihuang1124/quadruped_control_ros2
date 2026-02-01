//
// Created by lxq on 26-02-01.
//

#ifndef STATEFIXEDDOWNADJUSTABLELEG_H
#define STATEFIXEDDOWNADJUSTABLELEG_H

#include <vector>
#include "controller_common/FSM/FSMState.h"

class StateFixedDownAdjustableLeg final : public FSMState
{
public:
    explicit StateFixedDownAdjustableLeg(CtrlInterfaces& ctrl_interfaces,
                            const std::vector<double>& target_pos,
                            double kp,
                            double kd,
                            const double kp_special,  // 新增：特殊关节的 kp
                            const double kd_special  // 新增：特殊关节的 kd
                            );
    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;
private:
    double target_pos_[16] = {};
    double start_pos_[16] = {};
    rclcpp::Time start_time_;

    double kp_, kd_, kp_special_, kd_special_;

    double duration_ = 600; // steps
    double percent_ = 0; //%
    double phase = 0.0;
};


#endif //STATEFIXEDDOWNADJUSTABLELEG_H
