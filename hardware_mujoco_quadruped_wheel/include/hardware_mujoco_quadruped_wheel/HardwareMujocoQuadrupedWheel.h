//
// Created by ray on 25-8-15.
//

#pragma once

#include "hardware_interface/system_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "custom_msgs/msg/actuator_cmds.hpp"
#include "custom_msgs/msg/mujoco_msg.hpp"

class HardwareMujocoQuadrupedWheel final : public hardware_interface::SystemInterface
{
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
    int dof_manipulator_ = 8;
    int dof_quadruped_legs = 12;
    int dof_quadruped_wheel = 16;
    void imu_callback(const sensor_msgs::msg::Imu imu_state);
    void manipulator_joint_state_callback(const sensor_msgs::msg::JointState joint_state);

    void quadruped_joint_state_callback(const sensor_msgs::msg::JointState joint_state);

    // cmd
    std::unordered_map<std::string, double> joint_position_commands_;
    std::unordered_map<std::string, double> joint_velocity_commands_;
    std::unordered_map<std::string, double> joint_effort_commands_;
    std::unordered_map<std::string, double> joint_kp_commands_;
    std::unordered_map<std::string, double> joint_kd_commands_;

    // state
    std::unordered_map<std::string, double> joint_position_states_;
    std::unordered_map<std::string, double> joint_velocity_states_;
    std::unordered_map<std::string, double> joint_effort_states_;
    std::vector<double> imu_states_;

    /*node*/
    rclcpp::Node::SharedPtr node_;
    /*publisher*/
    rclcpp::Publisher<custom_msgs::msg::ActuatorCmds>::SharedPtr quadruped_actuator_cmd_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr manipulator_joint_cmd_publisher_;
    /*subscriber*/
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr manipulator_joint_state_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr quadruped_joint_state_subscriber_;
};
