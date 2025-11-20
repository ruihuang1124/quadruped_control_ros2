//
// Created by arclab-arcdog on 25-11-20.
//

#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/sample-models.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include <Eigen/Dense>
#include <memory>

class ArmController {
public:
    explicit ArmController();
    ~ArmController() = default;

    void initialize();
    void run();

private:
    // Pinocchio models
    pinocchio::Model model_;
    pinocchio::Data data_;

    // State variables
    struct RobotState {
        Eigen::Vector2f roll_pitch{0, 0};
        Eigen::Vector3f ang_vel{0, 0, 0};
        Eigen::Matrix<float, 18, 1> dof_pos{Eigen::Matrix<float, 18, 1>::Zero()};
        Eigen::Matrix<float, 18, 1> dof_vel{Eigen::Matrix<float, 18, 1>::Zero()};
        Eigen::Matrix<float, 12, 1> pre_action{Eigen::Matrix<float, 12, 1>::Zero()};
        Eigen::Matrix<float, 4, 1> foot_contact{Eigen::Matrix<float, 4, 1>::Zero()};

        // Command and goals
        Eigen::Vector3f cmd{Eigen::Vector3f::Zero()};
        Eigen::Vector3f ee_goal_cart{0.02, 0.0, 0.08};
        Eigen::Quaterniond ee_goal_quat{1.0, 0.0, 0.0, 0.0};
        Eigen::Vector3f ee_goal_sphere{Eigen::Vector3f::Zero()};
        Eigen::Matrix<float, 5, 1> gait{Eigen::Matrix<float, 5, 1>::Zero()};
        float gripper_pos{0.25};
    };

    RobotState state_;

    // Trajectory parameters
    Eigen::MatrixXd trajectory_coeffs_;
    double time_horizon_{0.0};
    double lerp_time_{0.01};
    int iteration_count_{0};

    // Constants
    static constexpr double CONTROL_FREQUENCY = 50.0;
    static constexpr double EPSILON = 1e-4;
    static constexpr int MAX_ITERATIONS = 10;
    static constexpr double DAMPING = 1e-3;
    static constexpr double TIME_STEP = 0.01;

    // Controller gains
    Eigen::VectorXd joint_kp_;

    // Methods
    void loadURDFModel(const std::string& urdf_path);

    // Control methods
    bool computeInverseKinematics(const pinocchio::SE3& desired_pose,
                                  Eigen::VectorXd& current_config,
                                  Eigen::VectorXd& target_config);
    void planTrajectory(const Eigen::VectorXd& target_positions,
                        const Eigen::VectorXd& current_positions,
                        const Eigen::VectorXd& current_velocities);
    Eigen::VectorXd evaluateTrajectory(double time) const;
    void SetJointCommands(const Eigen::VectorXd& target_positions,
                             const Eigen::VectorXd& nonlinear_effects);

    // Utility methods
    void initializeRobotConfiguration();
    void printRobotInfo();
};

#endif // ARM_CONTROLLER_H
