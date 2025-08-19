//
// Created by ray on 25-8-19.
//

#ifndef CTRLCOMPONENT_H
#define CTRLCOMPONENT_H
#include <model_based_controller/gait/WaveGenerator.h>

#include "BalanceCtrl.h"
#include "Estimator.h"

struct CtrlComponent {
    std::shared_ptr<QuadrupedRobot> robot_model_;
    std::shared_ptr<Estimator> estimator_;
    std::shared_ptr<BalanceCtrl> balance_ctrl_;
    std::shared_ptr<WaveGenerator> wave_generator_;

    CtrlComponent() = default;
};
#endif //CTRLCOMPONENT_H
