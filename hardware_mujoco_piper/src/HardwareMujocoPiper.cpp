//
// Created by biao on 24-9-9.
//

#include "hardware_mujoco_piper/HardwareMujocoPiper.h"
#include <rclcpp/logging.hpp>

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

using hardware_interface::return_type;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareMujocoPiper::on_init(
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
        // joint_effort_commands_[info.joints[i].name] = 0.0;
        // joint_kp_commands_[info.joints[i].name] = 0.0;
        // joint_kd_commands_[info.joints[i].name] = 0.0;
    }
    // imu_states_.resize(info.sensors[0].state_interfaces.size(), 0);
    // foot_contact_states_.resize(info.sensors[1].state_interfaces.size(), 0);

    node_ = rclcpp::Node::make_shared("ros2_control_mujoco");
    // subscription
    joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states_single", rclcpp::SensorDataQoS(), std::bind(&HardwareMujocoPiper::joint_state_callback, this, std::placeholders::_1));
    // imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    //     "imu_data", rclcpp::SensorDataQoS(), std::bind(&HardwareMujocoPiper::imu_callback, this, std::placeholders::_1));
    // foot_contact_state_subscriber_ = node_->create_subscription<custom_msgs::msg::MujocoMsg>(
    //     "mujoco_msg", rclcpp::SensorDataQoS(), std::bind(&HardwareMujocoPiper::foot_contact_callback, this, std::placeholders::_1));

    // publish
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    // actuator_cmd_publisher_ = node_->create_publisher<custom_msgs::msg::ActuatorCmds>("actuators_cmds", qos);

    joint_cmd_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_ctrl_single", qos);

    return SystemInterface::on_init(info);
}

std::vector<hardware_interface::StateInterface> HardwareMujocoPiper::export_state_interfaces()
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
    // for (size_t i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
    // {
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_states_[i]));
    // }

    // // force contact sensor
    // for (size_t i = 0; i < info_.sensors[1].state_interfaces.size(); i++)
    // {
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         info_.sensors[1].name, info_.sensors[1].state_interfaces[i].name, &foot_contact_states_[i]));
    // }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareMujocoPiper::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(info_.joints.size() * 2);

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "position", &joint_position_commands_[info_.joints[i].name]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "velocity", &joint_velocity_commands_[info_.joints[i].name]));
        // command_interfaces.emplace_back(hardware_interface::CommandInterface(
        //     info_.joints[i].name, "effort", &joint_effort_commands_[info_.joints[i].name]));
        // command_interfaces.emplace_back(hardware_interface::CommandInterface(
        //     info_.joints[i].name, "kp", &joint_kp_commands_[info_.joints[i].name]));
        // command_interfaces.emplace_back(hardware_interface::CommandInterface(
        //     info_.joints[i].name, "kd", &joint_kd_commands_[info_.joints[i].name]));
    }
    return command_interfaces;
}

return_type HardwareMujocoPiper::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

    if (rclcpp::ok())
    {
        rclcpp::spin_some(node_);
    }

    return return_type::OK;
}

return_type HardwareMujocoPiper::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // TODO: emplace_back or push_back
    custom_msgs::msg::ActuatorCmds actuator_cmds;
    sensor_msgs::msg::JointState joint_cmds;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        joint_cmds.name.push_back(info_.joints[i].name);
        joint_cmds.position.push_back(joint_position_commands_[info_.joints[i].name]);
        joint_cmds.velocity.push_back(joint_velocity_commands_[info_.joints[i].name]);
        // actuator_cmds.actuators_name.push_back(info_.joints[i].name);
        // actuator_cmds.pos.push_back(joint_position_commands_[info_.joints[i].name]);
        // actuator_cmds.vel.push_back(joint_velocity_commands_[info_.joints[i].name]);
        // actuator_cmds.torque.push_back(joint_effort_commands_[info_.joints[i].name]);
        // actuator_cmds.kp.push_back(joint_kp_commands_[info_.joints[i].name]);
        // actuator_cmds.kd.push_back(joint_kd_commands_[info_.joints[i].name]);
    }
    joint_cmd_publisher_->publish(joint_cmds);

    // {
    //     auto now = std::chrono::system_clock::now();
    //     auto now_time_t = std::chrono::system_clock::to_time_t(now);
    //     auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    //     std::tm now_tm;
    //     localtime_r(&now_time_t, &now_tm);
    //     std::ostringstream oss;
    //     oss << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S") << '.'
    //         << std::setfill('0') << std::setw(3) << now_ms.count();
    //     RCLCPP_INFO(node_->get_logger(), "Current time: %s", oss.str().c_str());
    // }

    return return_type::OK;
}

void HardwareMujocoPiper::joint_state_callback(const sensor_msgs::msg::JointState joint_state)
{
    for (size_t i = 0; i < joint_state.name.size(); i++)
    {
        joint_position_states_[joint_state.name[i]] = joint_state.position[i];
        joint_velocity_states_[joint_state.name[i]] = joint_state.velocity[i];
        joint_effort_states_[joint_state.name[i]] = joint_state.effort[i];
    }
}

// void HardwareMujocoPiper::foot_contact_callback(const custom_msgs::msg::MujocoMsg foot_contact_state)
// {
//     for (int i = 0; i < 4; i++)
//     {
//         foot_contact_states_[i] = foot_contact_state.contact_state[i];
//     }
// }

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(HardwareMujocoPiper, hardware_interface::SystemInterface)
