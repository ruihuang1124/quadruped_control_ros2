//
// Created by ray on 25-8-19.
//

#include "model_based_controller/FSM/StateFixedStand.h"

StateFixedStand::StateFixedStand(CtrlInterfaces &ctrl_interfaces, const std::vector<double> &target_pos,
                                 const double kp,
                                 const double kd)
    : BaseFixedStand(ctrl_interfaces, target_pos, kp, kd) {
}

FSMStateName StateFixedStand::checkChange() {
    if (percent_ < 1.5) {
        return FSMStateName::FIXEDSTAND;
    }
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 0:
            return FSMStateName::PASSIVE;
        case 1:
            return FSMStateName::FIXEDDOWN;
        case 3:
            return FSMStateName::FREESTAND;
        default:
            return FSMStateName::FIXEDSTAND;
    }
}
