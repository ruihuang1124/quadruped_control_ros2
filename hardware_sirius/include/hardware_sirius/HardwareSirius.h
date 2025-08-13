//
// Created by ray on 25-6-18.
//

#pragma once

#include "hardware_interface/system_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
// #include "custom_msgs/msg/robot_state.hpp"
// #include "custom_msgs/msg/robot_cmd.hpp"
#include "robot_interface/msg/robot_cmd.hpp"
#include "robot_interface/msg/robot_state.hpp"


class HardwareSirius final : public hardware_interface::SystemInterface
{
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
    void robot_state_callback(const robot_interface::msg::RobotState robot_state);

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
    rclcpp::Publisher<robot_interface::msg::RobotCMD>::SharedPtr robot_cmd_publisher_;
    /*subscriber*/
    rclcpp::Subscription<robot_interface::msg::RobotState>::SharedPtr robot_state_subscriber_;
};
