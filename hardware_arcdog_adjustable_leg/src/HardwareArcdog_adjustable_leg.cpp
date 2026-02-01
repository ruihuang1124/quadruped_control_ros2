//
// Created by biao on 24-9-9.
//

// #include "hardware_arcdog/HardwareArcdog.h"
#include "hardware_arcdog_adjustable_leg/HardwareArcdog_adjustable_leg.h"
#include <rclcpp/logging.hpp>

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

using hardware_interface::return_type;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareArcdog_adjustable_leg::on_init(
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
    // foot_contact_states_.resize(info.sensors[1].state_interfaces.size(), 0);

    limit_abad_.min = -100.80; 
    limit_abad_.max =  100.80;
    limit_hip_.min = -100.0;
    limit_hip_.max =  400.0;
    limit_knee_.min = -300.8;
    limit_knee_.max = 110.5;

    node_ = rclcpp::Node::make_shared("ros2_control_arcdog");
    // subscription
    // joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    //     "joint_states", rclcpp::SensorDataQoS(), std::bind(&HardwareArcdog::joint_state_callback, this, std::placeholders::_1));
    imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(), std::bind(&HardwareArcdog_adjustable_leg::imu_callback, this, std::placeholders::_1));
    // publish
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    // actuator_cmd_publisher_ = node_->create_publisher<custom_msgs::msg::ActuatorCmds>("actuators_cmds", qos);
    joint_commands_pub_ = node_->create_publisher<custom_msgs::msg::JointCommands>(
    "joint_commands",  // 话题名称
    qos  // QoS 队列大小
    );

    motor_mode_ = 1;
    motor_activation_server_ = node_->create_service<custom_msgs::srv::ExecuteMotorActivation>(
        "motor_activation", std::bind(&HardwareArcdog_adjustable_leg::motor_activation_callback, this, std::placeholders::_1, std::placeholders::_2));
    iterations_ = 0;
    tty_descriptor_ = usb_driver_start();

    return SystemInterface::on_init(info);
}

std::vector<hardware_interface::StateInterface> HardwareArcdog_adjustable_leg::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(info_.joints.size() * 3 + info_.sensors.size() * 2);

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

    // // force contact sensor
    // for (size_t i = 0; i < info_.sensors[1].state_interfaces.size(); i++)
    // {
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         info_.sensors[1].name, info_.sensors[1].state_interfaces[i].name, &foot_contact_states_[i]));
    // }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareArcdog_adjustable_leg::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(info_.joints.size() * 5);

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

return_type HardwareArcdog_adjustable_leg::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // read motor states through tty and the imu info will be updated with imucallback().
    custom_msgs::msg::JointStates* joints_data = get_joint_states_msgtype();

    if (joints_data != nullptr) {
        check_joint_limits(joints_data);
    }

    if (rclcpp::ok())
    {
        for (size_t i = 0; i < LEG_AMOUNT; i++)
        {
            joint_position_states_[info_.joints[0 + i*4].name] = joints_data->q_abad[i];
            joint_position_states_[info_.joints[1 + i*4].name] = joints_data->q_hip[i];
            joint_position_states_[info_.joints[2 + i*4].name] = joints_data->q_knee[i];
            joint_position_states_[info_.joints[3 + i*4].name] = joints_data->q_prismatic[i];
            joint_velocity_states_[info_.joints[0 + i*4].name] = joints_data->qd_abad[i];
            joint_velocity_states_[info_.joints[1 + i*4].name] = joints_data->qd_hip[i];
            joint_velocity_states_[info_.joints[2 + i*4].name] = joints_data->qd_knee[i];
            joint_velocity_states_[info_.joints[3 + i*4].name] = joints_data->qd_prismatic[i];
            joint_effort_states_[info_.joints[0 + i*4].name] = joints_data->tau_abad[i];
            joint_effort_states_[info_.joints[1 + i*4].name] = joints_data->tau_hip[i];
            joint_effort_states_[info_.joints[2 + i*4].name] = joints_data->tau_knee[i];
            joint_effort_states_[info_.joints[3 + i*4].name] = joints_data->tau_prismatic[i];
        }
        rclcpp::spin_some(node_);
    }

    return return_type::OK;
}

return_type HardwareArcdog_adjustable_leg::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // TODO: emplace_back or push_back
    custom_msgs::msg::JointCommands* joints_command = get_joint_commands_msgtype();
    CAN_HOST_DATA *can_host_cmd = get_can_host_data();
    for (size_t i = 0; i < LEG_AMOUNT; i++)
    {
        joints_command->kp_abad[i] = joint_kp_commands_[info_.joints[0 + i*4].name];
        joints_command->kp_hip[i] = joint_kp_commands_[info_.joints[1 + i*4].name];
        joints_command->kp_knee[i] = joint_kp_commands_[info_.joints[2 + i*4].name];
        joints_command->kp_prismatic[i] = joint_kp_commands_[info_.joints[3 + i*4].name];

        joints_command->kd_abad[i] = joint_kd_commands_[info_.joints[0 + i*4].name];
        joints_command->kd_hip[i] = joint_kd_commands_[info_.joints[1 + i*4].name];
        joints_command->kd_knee[i] = joint_kd_commands_[info_.joints[2 + i*4].name];
        joints_command->kd_prismatic[i] = joint_kd_commands_[info_.joints[3 + i*4].name];

        joints_command->q_des_abad[i] = joint_position_commands_[info_.joints[0 + i*4].name];
        joints_command->q_des_hip[i] = joint_position_commands_[info_.joints[1 + i*4].name];
        joints_command->q_des_knee[i] = joint_position_commands_[info_.joints[2 + i*4].name];
        joints_command->q_des_prismatic[i] = joint_position_commands_[info_.joints[3 + i*4].name];

        joints_command->qd_des_abad[i] = joint_velocity_commands_[info_.joints[0 + i*4].name];
        joints_command->qd_des_hip[i] = joint_velocity_commands_[info_.joints[1 + i*4].name];
        joints_command->qd_des_knee[i] = joint_velocity_commands_[info_.joints[2 + i*4].name];
        joints_command->qd_des_prismatic[i] = joint_velocity_commands_[info_.joints[3 + i*4].name];

        joints_command->tau_abad_ff[i] = joint_effort_commands_[info_.joints[0 + i*4].name];
        joints_command->tau_hip_ff[i] = joint_effort_commands_[info_.joints[1 + i*4].name];
        joints_command->tau_knee_ff[i] = joint_effort_commands_[info_.joints[2 + i*4].name];
        joints_command->tau_prismatic_ff[i] = joint_effort_commands_[info_.joints[3 + i*4].name];
    }

    auto ros_msg = std::make_unique<custom_msgs::msg::JointCommands>();
    ros_msg->header.stamp = node_->now();
    for (size_t i = 0; i < LEG_AMOUNT; i++) {
        // 位置命令
        ros_msg->q_des_abad[i] = joint_position_commands_[info_.joints[0 + i*4].name];
        ros_msg->q_des_hip[i] = joint_position_commands_[info_.joints[1 + i*4].name];
        ros_msg->q_des_knee[i] = joint_position_commands_[info_.joints[2 + i*4].name];
        ros_msg->q_des_prismatic[i] = joint_position_commands_[info_.joints[3 + i*4].name];
        
        // 速度命令
        ros_msg->qd_des_abad[i] = joint_velocity_commands_[info_.joints[0 + i*4].name];
        ros_msg->qd_des_hip[i] = joint_velocity_commands_[info_.joints[1 + i*4].name];
        ros_msg->qd_des_knee[i] = joint_velocity_commands_[info_.joints[2 + i*4].name];
        ros_msg->qd_des_prismatic[i] = joint_velocity_commands_[info_.joints[3 + i*4].name];
        
        // 力矩前馈
        ros_msg->tau_abad_ff[i] = joint_effort_commands_[info_.joints[0 + i*4].name];
        ros_msg->tau_hip_ff[i] = joint_effort_commands_[info_.joints[1 + i*4].name];
        ros_msg->tau_knee_ff[i] = joint_effort_commands_[info_.joints[2 + i*4].name];
        ros_msg->tau_prismatic_ff[i] = joint_effort_commands_[info_.joints[3 + i*4].name];

        // PID增益
        ros_msg->kp_abad[i] = joint_kp_commands_[info_.joints[0 + i*4].name];
        ros_msg->kp_hip[i] = joint_kp_commands_[info_.joints[1 + i*4].name];
        ros_msg->kp_knee[i] = joint_kp_commands_[info_.joints[2 + i*4].name];
        ros_msg->kp_prismatic[i] = joint_kp_commands_[info_.joints[3 + i*4].name];
        ros_msg->kd_abad[i] = joint_kd_commands_[info_.joints[0 + i*4].name];
        ros_msg->kd_hip[i] = joint_kd_commands_[info_.joints[1 + i*4].name];
        ros_msg->kd_knee[i] = joint_kd_commands_[info_.joints[2 + i*4].name];
        ros_msg->kd_prismatic[i] = joint_kd_commands_[info_.joints[3 + i*4].name];
        // ... 其他增益命令 ...
    }
    joint_commands_pub_->publish(std::move(ros_msg));

    bool motor_mode_flag = false; // true if we want the motor move. activated_values input TODO.
    switch (motor_mode_) {
    case 0:// motors are ready to move, after activated:
        // printf("Motor ready to received move cmd!\n");
        motor_mode_flag = true;
        break;
    case 1:
        // printf("Motor deactivate!\n");
        motor_mode_flag = false;
        for (int leg = 0; leg < 4; leg++) {
            for(int i=0;i<7;i++)
            {
                can_host_cmd[leg].dataA[i]=0xFF;
                can_host_cmd[leg].dataB[i]=0xFF;
                can_host_cmd[leg].dataC[i]=0xFF;
                can_host_cmd[leg].dataD[i]=0xFF;
            }
            for(int i=7;i<8;i++)
            {
                can_host_cmd[leg].dataA[i]=0xFD;
                can_host_cmd[leg].dataB[i]=0xFD;
                can_host_cmd[leg].dataC[i]=0xFD;
                can_host_cmd[leg].dataD[i]=0xFD;
            }
        }
        break;
    case 8:
        iterations_++;
        // printf("Motor activate!\n");
        motor_mode_flag = false;
        for (int leg = 0; leg < 4; leg++) {
            for(int i=0;i<7;i++)
            {
                can_host_cmd[leg].dataA[i]=0xFF;
                can_host_cmd[leg].dataB[i]=0xFF;
                can_host_cmd[leg].dataC[i]=0xFF;
                can_host_cmd[leg].dataD[i]=0xFF;
            }
            for(int i=7;i<8;i++)
            {
                can_host_cmd[leg].dataA[i]=0xFC;
                can_host_cmd[leg].dataB[i]=0xFC;
                can_host_cmd[leg].dataC[i]=0xFC;
                can_host_cmd[leg].dataD[i]=0xFC;
            }
        }
        break;
    }
    usb_driver_run(tty_descriptor_,motor_mode_flag,iterations_);
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

void HardwareArcdog_adjustable_leg::imu_callback(const sensor_msgs::msg::Imu imu_state)
{
    // std::cerr<<"received imu callback"<<std::endl;
    // // 打印方向（四元数）
    // std::cout << "Orientation: [w: " << imu_state.orientation.w 
    //           << ", x: " << imu_state.orientation.x 
    //           << ", y: " << imu_state.orientation.y 
    //           << ", z: " << imu_state.orientation.z << "]" << std::endl;

    // // 打印角速度
    // std::cout << "Angular Velocity: [x: " << imu_state.angular_velocity.x 
    //           << ", y: " << imu_state.angular_velocity.y 
    //           << ", z: " << imu_state.angular_velocity.z << "]" << std::endl;

    // // 打印线性加速度
    // std::cout << "Linear Acceleration: [x: " << imu_state.linear_acceleration.x 
    //           << ", y: " << imu_state.linear_acceleration.y 
    //           << ", z: " << imu_state.linear_acceleration.z << "]" << std::endl;
    imu_states_[0] = imu_state.orientation.w;
    imu_states_[1] = imu_state.orientation.x;
    imu_states_[2] = imu_state.orientation.y;
    imu_states_[3] = imu_state.orientation.z;
    imu_states_[4] = imu_state.angular_velocity.x;
    imu_states_[5] = imu_state.angular_velocity.y;
    imu_states_[6] = imu_state.angular_velocity.z;
    imu_states_[7] = imu_state.linear_acceleration.x;
    imu_states_[8] = imu_state.linear_acceleration.y;
    imu_states_[9] = imu_state.linear_acceleration.z;
}
// void HardwareArcdog::joint_state_callback(const sensor_msgs::msg::JointState joint_state)
// {
//     for (size_t i = 0; i < joint_state.name.size(); i++)
//     {
//         joint_position_states_[joint_state.name[i]] = joint_state.position[i];
//         joint_velocity_states_[joint_state.name[i]] = joint_state.velocity[i];
//         joint_effort_states_[joint_state.name[i]] = joint_state.effort[i];
//     }
// }

void HardwareArcdog_adjustable_leg::motor_activation_callback(const custom_msgs::srv::ExecuteMotorActivation::Request::SharedPtr req,
                                               const custom_msgs::srv::ExecuteMotorActivation::Response::SharedPtr res)
{
    if (req->motor_mode == 8)
    {
        RCLCPP_WARN(node_->get_logger(), "robot motors activated! be careful!");
    } else if (req->motor_mode == 0)
    {
        RCLCPP_WARN(node_->get_logger(), "robot motors activated and are allow actuated by the commands!");
        // printf("\n\n\n\n\n\n\n\n"); 
    } else if (req->motor_mode == 1)
    {
        RCLCPP_WARN(node_->get_logger(), "robot motors deactivated.");
    } else
    {
        RCLCPP_WARN(node_->get_logger(), "Nothing changed, 1 for motors deactivated, 8 for motors activated and 0 for controlling robots");
    }
    motor_mode_ = req->motor_mode;
    res->result_status = res->SUCCEEDED;
}

void HardwareArcdog_adjustable_leg::check_joint_limits(const custom_msgs::msg::JointStates* joints_data)
{
    
    // 缓冲检查，如果运行周期小于 1600 次，直接跳过检查，不执行后续逻辑
    if (iterations_ <= 1600) {
        return;
    }

    // 如果电机已经是失能状态(1)，则不需要重复检查和报错
    if (motor_mode_ == 1) {
        return;
    }

    bool limit_violated = false;
    std::string violated_joint_name = "";
    double violated_value = 0.0;

    // 遍历 4 条腿
    for (size_t i = 0; i < LEG_AMOUNT; i++)
    {
        // 1. 检查 Abad (侧摆)
        if (joints_data->q_abad[i] < limit_abad_.min || joints_data->q_abad[i] > limit_abad_.max) {
            limit_violated = true;
            violated_joint_name = "Leg " + std::to_string(i) + " Abad";
            violated_value = joints_data->q_abad[i];
            break; 
        }

        // 2. 检查 Hip (大腿)
        if (joints_data->q_hip[i] < limit_hip_.min || joints_data->q_hip[i] > limit_hip_.max) {
            limit_violated = true;
            violated_joint_name = "Leg " + std::to_string(i) + " Hip";
            violated_value = joints_data->q_hip[i];
            break;
        }

        // 3. 检查 Knee (膝盖)
        if (joints_data->q_knee[i] < limit_knee_.min || joints_data->q_knee[i] > limit_knee_.max) {
            limit_violated = true;
            violated_joint_name = "Leg " + std::to_string(i) + " Knee";
            violated_value = joints_data->q_knee[i];
            break;
        }
    }

    if (limit_violated)
    {
        // 强制切换到失能模式
        motor_mode_ = 1;
        
        // 打印红色致命错误日志
        RCLCPP_FATAL(node_->get_logger(), 
            "JOINT LIMIT VIOLATED! Emergency Stop Triggered. Joint: %s, Value: %f", 
            violated_joint_name.c_str(), violated_value);
            
        // 可选：你也可以在这里清空所有的 command，防止恢复后瞬间跳变
        // for (auto & cmd : joint_effort_commands_) cmd.second = 0.0;
    }
}


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(HardwareArcdog_adjustable_leg, hardware_interface::SystemInterface)
