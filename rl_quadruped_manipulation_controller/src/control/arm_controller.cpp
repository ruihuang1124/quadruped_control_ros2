//
// Created by arclab-arcdog on 25-11-20.
//

#include "rl_quadruped_manipulation_controller/control/arm_controller.h"

// Debug macros
#define PRINT_VAR(var) std::cout << #var << ": " << var << std::endl;
#define PRINT_VAR_T(var) std::cout << #var << ": " << var.transpose() << std::endl;

ArmController::ArmController()
{
    joint_kp_.resize(6);
    joint_kp_ << 400.0, 400.0, 400.0, 400.0, 400.0, 100.0;
}

void ArmController::initialize()
{
    // Load URDF model
    const std::string urdf_filename = "urdf/sirius_piper.urdf";
    loadURDFModel(urdf_filename);

    // Initialize robot configuration
    initializeRobotConfiguration();

    // Print robot information
    printRobotInfo();
}

void ArmController::loadURDFModel(const std::string& urdf_path)
{
    pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model_);
    data_ = pinocchio::Data(model_);
    std::cout << "Loaded model: " << model_.name << std::endl;
}


void ArmController::initializeRobotConfiguration()
{
    Eigen::VectorXd q(model_.nq), dq(model_.nv);
    dq.setZero();

    // Initial configuration
    q << 0, 0, 0.45, 0, 0, 0, 1,
        0.1, -0.8, 1.6,
        0.1, 0.8, -1.6,
        -0.1, -0.8, 1.6,
        -0.1, 0.8, -1.6,
        0, 0, -1.0, 1.0, 0, 0;

    std::cout << "Initial configuration: " << q.transpose() << std::endl;

    // Perform initial forward kinematics
    pinocchio::forwardKinematics(model_, data_, q);
}

void ArmController::printRobotInfo()
{
    // Print joint placements
    for (pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id)
    {
        std::cout << std::setw(24) << std::left << model_.names[joint_id]
            << ": " << std::fixed << std::setprecision(2)
            << data_.oMi[joint_id].translation().transpose() << std::endl;
    }

    // Update and print frame information
    pinocchio::FrameIndex ee_id = model_.getFrameId("gripper_base");
    pinocchio::updateFramePlacement(model_, data_, ee_id);

    for (pinocchio::FrameIndex frame_id = 0; frame_id < (pinocchio::FrameIndex)model_.nframes; ++frame_id)
    {
        std::cout << std::setw(3) << frame_id << ": "
            << std::setw(24) << std::left << model_.frames[frame_id].name
            << ": " << std::fixed << data_.oMf[frame_id].translation().transpose() << std::endl;
    }
}


bool ArmController::computeInverseKinematics(const pinocchio::SE3& desired_pose,
                                             Eigen::VectorXd& current_config,
                                             Eigen::VectorXd& target_config)
{
    pinocchio::FrameIndex ee_id = model_.getFrameId("gripper_base");
    pinocchio::Data::Matrix6x J(6, model_.nv);
    J.setZero();

    int iteration = 0;
    double error_norm = 100.0;
    Eigen::VectorXd error_vector(6), config_error(model_.nv);
    while (iteration < MAX_ITERATIONS && error_norm > EPSILON)
    {
        pinocchio::forwardKinematics(model_, data_, target_config);
        pinocchio::updateFramePlacement(model_, data_, ee_id);

        const pinocchio::SE3 current_to_desired = data_.oMf[ee_id].actInv(desired_pose);
        error_vector = pinocchio::log6(current_to_desired).toVector();
        pinocchio::computeFrameJacobian(model_, data_, target_config, ee_id,
                                        pinocchio::LOCAL, J);
        // Zero out base motion components
        J.block(0, 0, 6, 6).setZero();
        pinocchio::Data::Matrix6 Jlog;
        pinocchio::Jlog6(current_to_desired.inverse(), Jlog);
        J = -Jlog * J;

        // Damped least squares
        pinocchio::Data::Matrix6 JJt;
        JJt = J * J.transpose();
        JJt.diagonal().array() += DAMPING;
        config_error.noalias() = -J.transpose() * JJt.ldlt().solve(error_vector);
        target_config = pinocchio::integrate(model_, target_config, config_error * TIME_STEP);
        iteration++;
        error_norm = error_vector.norm();
    }
    return error_norm <= EPSILON;
}

void ArmController::planTrajectory(const Eigen::VectorXd& target_positions,
                                   const Eigen::VectorXd& current_positions,
                                   const Eigen::VectorXd& current_velocities)
{
    const int num_dimensions = 6;
    const double max_velocity = 2.0;
    time_horizon_ = (target_positions - current_positions).norm() / max_velocity;
    if (time_horizon_ < 0.002) time_horizon_ = 0.002;
    trajectory_coeffs_.resize(num_dimensions, 6);
    for (int dim = 0; dim < num_dimensions; ++dim)
    {
        Eigen::Matrix<double, 6, 6> constraint_matrix;
        constraint_matrix <<
            1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 2, 0, 0, 0,
            1, time_horizon_, std::pow(time_horizon_, 2), std::pow(time_horizon_, 3), std::pow(time_horizon_, 4),
            std::pow(time_horizon_, 5),
            0, 1, 2 * time_horizon_, 3 * std::pow(time_horizon_, 2), 4 * std::pow(time_horizon_, 3), 5 * std::pow(
                time_horizon_, 4),
            0, 0, 2, 6 * time_horizon_, 12 * std::pow(time_horizon_, 2), 20 * std::pow(time_horizon_, 3);

        Eigen::VectorXd constraints(6);
        constraints << current_positions(dim), current_velocities(dim), 0, target_positions(dim), 0, 0;
        trajectory_coeffs_.row(dim).noalias() = constraint_matrix.colPivHouseholderQr().solve(constraints);
    }
}

Eigen::VectorXd ArmController::evaluateTrajectory(double time) const
{
    Eigen::VectorXd time_powers(6);
    for (int power = 0; power < 6; ++power)
    {
        time_powers(power) = std::pow(time, power);
    }
    return trajectory_coeffs_ * time_powers;
}


void ArmController::run()
{
    Eigen::VectorXd current_q(model_.nq), target_q_ik(model_.nq);
    Eigen::VectorXd current_dq(model_.nv);
    // Update current state from received messages
    current_q.segment(19, 6) = state_.dof_pos.segment(12, 6).cast<double>();
    current_dq.segment(18, 6) = state_.dof_vel.segment(12, 6).cast<double>();
    pinocchio::SE3 desired_ee_pose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.03, 0., 0.8));
    // Compute inverse kinematics periodically

    std::cout << "ee_goal_cart: " << state_.ee_goal_cart.transpose() << std::endl;
    desired_ee_pose.translation() = (state_.ee_goal_cart + state_.cmd).cast<double>();
    desired_ee_pose.rotation() = state_.ee_goal_quat.toRotationMatrix();
    target_q_ik = current_q;
    if (computeInverseKinematics(desired_ee_pose, current_q, target_q_ik))
    {
        std::cout << "IK solution found: "
            << target_q_ik.segment(19, 6).transpose() << std::endl;;

        // Replan trajectory if needed
        if (lerp_time_ >= time_horizon_)
        {
            planTrajectory(target_q_ik.segment(19, 6),
                           current_q.segment(19, 6),
                           current_dq.segment(18, 6));
            lerp_time_ = 0.0;
        }
    }
    // Advance trajectory time
    lerp_time_ += 1.0 / CONTROL_FREQUENCY;
    // Compute nonlinear effects compensation
    auto nonlinear_compensation = pinocchio::nonLinearEffects(model_, data_,
                                                              current_q, current_dq)
                                  .segment(18, 6).cwiseQuotient(joint_kp_);
    // Get current target positions from trajectory
    auto target_positions = evaluateTrajectory(lerp_time_);
    // Publish commands
    SetJointCommands(target_positions, nonlinear_compensation);
}
