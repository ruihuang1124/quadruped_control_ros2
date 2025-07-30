//
// Created by tlab-uav on 24-9-6.
//

#ifndef STATEPASSIVEADJUSTABLELEG_H
#define STATEPASSIVEADJUSTABLELEG_H
#include "controller_common/FSM/FSMState.h"

class StatePassiveAdjustableLeg final : public FSMState
{
public:
    explicit StatePassiveAdjustableLeg(CtrlInterfaces& ctrl_interfaces);

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;
};


#endif //STATEPASSIVEADJUSTABLELEG_H
