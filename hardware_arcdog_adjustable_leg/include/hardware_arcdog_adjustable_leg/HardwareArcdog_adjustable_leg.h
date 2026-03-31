//
// Created by biao on 24-9-9.
//

#pragma once

#include "hardware_interface/system_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "custom_msgs/msg/actuator_cmds.hpp"
#include "custom_msgs/msg/mujoco_msg.hpp"
// #include "rt_usb_cdc.h"
#include "rt_usb_cdc_adjustable_leg.h"
#include "custom_msgs/srv/execute_motor_activation.hpp"

#include "custom_msgs/msg/joint_commands.hpp"

class HardwareArcdog_adjustable_leg final : public hardware_interface::SystemInterface
{
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
    int tty_descriptor_;
    int iterations_;
    int motor_mode_;
    // bool motor_activation_flag_;
    struct JointLimit {
        double min;
        double max;
    };
    JointLimit limit_abad_;
    JointLimit limit_hip_;
    JointLimit limit_knee_;
    JointLimit limit_prismatic_; // 新增 prismatic 的限位结构体
    
    // 用于过滤限位噪音的计时变量
    bool is_violating_limits_ = false;      // 标记当前是否处于超限状态
    rclcpp::Time violation_start_time_;     // 记录开始超限的时间戳

    void imu_callback(const sensor_msgs::msg::Imu imu_state);
    void motor_activation_callback(const custom_msgs::srv::ExecuteMotorActivation::Request::SharedPtr req,
                                   const custom_msgs::srv::ExecuteMotorActivation::Response::SharedPtr res);
    // void joint_state_callback(const sensor_msgs::msg::JointState joint_state);
    void check_joint_limits(const custom_msgs::msg::JointStates* joints_data);

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
    // std::vector<double> foot_contact_states_;

    /*node*/
    rclcpp::Node::SharedPtr node_;
    /*publisher*/
    // rclcpp::Publisher<custom_msgs::msg::ActuatorCmds>::SharedPtr actuator_cmd_publisher_;
    rclcpp::Publisher<custom_msgs::msg::JointCommands>::SharedPtr joint_commands_pub_;
    /*subscriber*/
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

    rclcpp::Service<custom_msgs::srv::ExecuteMotorActivation>::SharedPtr motor_activation_server_;
    // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
};
