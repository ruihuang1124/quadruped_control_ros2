//
// Created by ray on 2025-07-25.
//

#ifndef QMCtrlComponent_H
#define QMCtrlComponent_H

#include "Estimator.h"
#include <rclcpp_lifecycle/lifecycle_node.hpp>

struct CtrlComponent {

    bool enable_estimator_ = false;
    std::shared_ptr<QuadrupedRobot> robot_model_;
    std::shared_ptr<Estimator> estimator_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

    CtrlComponent() = default;
};

#endif //QMCtrlComponent_H
