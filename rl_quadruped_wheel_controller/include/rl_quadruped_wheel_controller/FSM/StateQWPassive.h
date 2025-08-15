//
// Created by ray on 2025-07-25.
//

#ifndef STATEQMPASSIVE_H
#define STATEQMPASSIVE_H
#include <controller_common/FSM/FSMState.h>

class StateQWPassive final : public FSMState
{
public:
    explicit StateQWPassive(CtrlInterfaces& ctrl_interfaces);

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;
};


#endif //STATEQMPASSIVE_H
