//
// Created by ray on 2025-07-25.
//

#ifndef STATQMEFIXEDDOWN_H
#define STATEQMFIXEDDOWN_H

#include <controller_common/FSM/FSMState.h>

class StateQWFixedDown final : public FSMState
{
public:
    explicit StateQWFixedDown(CtrlInterfaces& ctrl_interfaces,
                            const std::vector<double>& target_pos,
                            double kp,
                            double kd
    );

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    int num_joints_ = 16;
    double target_pos_[16] = {};
    double start_pos_[16] = {};
    rclcpp::Time start_time_;

    double kp_, kd_;

    double duration_ = 600; // steps
    double percent_ = 0; //%
    double phase = 0.0;
};


#endif //STATEQMFIXEDDOWN_H
