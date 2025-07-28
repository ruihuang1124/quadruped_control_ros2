//
// Created by ray on 2025-07-25.
//

#ifndef STATEQMPASSIVE_H
#define STATEQMPASSIVE_H
#include <controller_common/FSM/FSMState.h>

class StateQMPassive final : public FSMState
{
public:
    explicit StateQMPassive(CtrlInterfaces& ctrl_interfaces);

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;
};


#endif //STATEQMPASSIVE_H
