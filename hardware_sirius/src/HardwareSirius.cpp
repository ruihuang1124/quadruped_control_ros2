//
// Created by biao on 24-9-9.
//

#include "hardware_sirius/HardwareSirius.h"
#include <rclcpp/logging.hpp>

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

using hardware_interface::return_type;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareSirius::on_init(
    const hardware_interface::HardwareInfo &info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < info.joints.size(); i++)
    {
        joint_position_states_[info.joints[i].name] = 0.0;
        joint_velocity_states_[info.joints[i].name] = 0.0;
        joint_effort_states_[info.joints[i].name] = 0.0;
        joint_position_commands_[info.joints[i].name] = 0.0;
        joint_velocity_commands_[info.joints[i].name] = 0.0;
        joint_effort_commands_[info.joints[i].name] = 0.0;
        joint_kp_commands_[info.joints[i].name] = 0.0;
        joint_kd_commands_[info.joints[i].name] = 0.0;
    }
    imu_states_.resize(info.sensors[0].state_interfaces.size(), 0);

    node_ = rclcpp::Node::make_shared("ros2_control_hardware_sirius");
    // subscription
   robot_state_subscriber_ = node_->create_subscription<custom_msgs::msg::RobotState>(
        "ROS2_Robot_State", rclcpp::SensorDataQoS(),
        std::bind(&HardwareSirius::robot_state_callback, this, std::placeholders::_1));

    // publish
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    robot_cmd_publisher_ = node_->create_publisher<custom_msgs::msg::RobotCMD>("RobotCMD", qos);

    return SystemInterface::on_init(info);
}

std::vector<hardware_interface::StateInterface> HardwareSirius::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(info_.joints.size() * 2);

    // joint state
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "position", &joint_position_states_[info_.joints[i].name]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "velocity", &joint_velocity_states_[info_.joints[i].name]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "effort", &joint_effort_states_[info_.joints[i].name]));
    }

    // imu sensor
    for (size_t i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_states_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareSirius::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(info_.joints.size() * 2);

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "position", &joint_position_commands_[info_.joints[i].name]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "velocity", &joint_velocity_commands_[info_.joints[i].name]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "effort", &joint_effort_commands_[info_.joints[i].name]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "kp", &joint_kp_commands_[info_.joints[i].name]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "kd", &joint_kd_commands_[info_.joints[i].name]));
    }
    return command_interfaces;
}

return_type HardwareSirius::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

    if (rclcpp::ok())
    {
        rclcpp::spin_some(node_);
    }

    return return_type::OK;
}

return_type HardwareSirius::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // TODO: emplace_back or push_back
    custom_msgs::msg::RobotCMD robot_cmds;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        robot_cmds.q[i] = joint_position_commands_[info_.joints[i].name]; // check the order TODO.
        robot_cmds.qd[i] = joint_velocity_commands_[info_.joints[i].name];
        robot_cmds.kp[i] = joint_kp_commands_[info_.joints[i].name];
        robot_cmds.kd[i] = joint_kd_commands_[info_.joints[i].name];
        robot_cmds.tau_ff[i] = joint_effort_commands_[info_.joints[i].name];
    }
    robot_cmd_publisher_->publish(robot_cmds);
    return return_type::OK;
}


void HardwareSirius::robot_state_callback(const custom_msgs::msg::RobotState robot_state) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
        joint_position_states_[info_.joints[i].name] = robot_state.q[i]; // check the order TODO.
        joint_velocity_states_[info_.joints[i].name] = robot_state.qd[i];
    }
    imu_states_[0] = robot_state.quat[0]; // check if quad[0] is quad.w or not TODO.
    for (size_t i = 0; i < 3; i++) {
        imu_states_[i + 1] = robot_state.quat[i];
        imu_states_[i + 4] = robot_state.gyro[i];
        imu_states_[i + 7] = robot_state.acc[i];
    }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(HardwareSirius, hardware_interface::SystemInterface)
