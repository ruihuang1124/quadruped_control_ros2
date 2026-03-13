//
// Created by qiayuan on 2022/7/26.
//

#pragma once

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_mpc/SystemObservation.h>
#include <custom_msgs/msg/user_cmds.hpp>
#include "ocs2_quadruped_controller/control/CtrlComponent.h"

namespace ocs2::legged_robot
{
    class SafetyChecker
    {
    public:
        explicit SafetyChecker(const CentroidalModelInfo &info) : info_(info)
        {
        }

        [[nodiscard]] bool check(const SystemObservation &observation, const vector_t & /*optimized_state*/,
                                 const vector_t & /*optimized_input*/, CtrlComponent& ctrl_comp) const
        {
            return (checkOrientation(observation) && checkPassiveState(ctrl_comp));
        }

    protected:
        [[nodiscard]] bool checkOrientation(const SystemObservation &observation) const
        {
            if (vector_t pose = centroidal_model::getBasePose(observation.state, info_);
                pose(5) > M_PI_2 || pose(5) < -M_PI_2)
            {
                std::cerr << "[SafetyChecker] Orientation safety check failed!" << std::endl;
                return false;
            }
            return true;
        }

        [[nodiscard]] bool checkPassiveState(CtrlComponent& ctrl_comp) const
        {
            bool is_passive = false;
            {
                std::lock_guard<std::mutex> lock(ctrl_comp.user_cmds_mutex_);
                is_passive = ctrl_comp.user_cmds_.passive_enable;
            }
            if (is_passive)
            {
                std::cerr << "[SafetyChecker] Passive command received!" << std::endl;
                return false;
            }
            return true;
        }
        const CentroidalModelInfo &info_;
    };
} // namespace legged
