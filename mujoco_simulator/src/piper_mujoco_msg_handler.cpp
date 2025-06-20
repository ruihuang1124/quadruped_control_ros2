#include "piper_mujoco_msg_handler.h"
#include <algorithm>
#include "sensor_msgs/image_encodings.hpp"

namespace ArcLab
{
    PiperMujocoMsgHandler::PiperMujocoMsgHandler(mj::Simulate *sim)
        : Node("PiperMujocoMsgHandler", rclcpp::NodeOptions().use_intra_process_comms(true)),
          sim_(sim)
    {
        this->declare_parameter<std::string>("xml_file_path", "path/to/your/xml");
        this->get_parameter("xml_file_path", xml_file_path_);
        std::cout << "xml file path:" << xml_file_path_ << std::endl;

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        // imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", qos);
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states_single", qos);
        // mujoco_msg_publisher_ = this->create_publisher<custom_msgs::msg::MujocoMsg>("mujoco_msg", qos);

        timers_.emplace_back(this->create_wall_timer(1ms, std::bind(&PiperMujocoMsgHandler::publish_mujoco_callback, this)));


        joint_cmd_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_ctrl_single",
            qos,
            std::bind(&PiperMujocoMsgHandler::joint_cmd_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Start PiperMujocoMsgHandler ...");
        sim_->uiloadrequest.fetch_add(1);
    }

    PiperMujocoMsgHandler::~PiperMujocoMsgHandler()
    {
        RCLCPP_INFO(this->get_logger(), "close node ...");
    }

    void PiperMujocoMsgHandler::publish_mujoco_callback()
    {
        if (sim_->d_ != nullptr)
        {
            const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
            joint_callback();
        }
    }

    void PiperMujocoMsgHandler::joint_callback()
    {
        if (sim_->d_ != nullptr)
        {
            sensor_msgs::msg::JointState jointState;
            jointState.header.frame_id = &sim_->m_->names[0];
            jointState.header.stamp = rclcpp::Clock().now();
            for (int i = 0; i < sim_->m_->njnt; i++)
            {
                if (sim_->m_->jnt_type[i] == mjtJoint::mjJNT_HINGE)
                {
                    std::string jnt_name(mj_id2name(sim_->m_, mjtObj::mjOBJ_JOINT, i));
                    jointState.name.emplace_back(jnt_name);
                    jointState.position.push_back(sim_->d_->qpos[sim_->m_->jnt_qposadr[i]]);
                    jointState.velocity.push_back(sim_->d_->qvel[sim_->m_->jnt_dofadr[i]]);
                    jointState.effort.push_back(sim_->d_->qfrc_actuator[sim_->m_->jnt_dofadr[i]]);
                }
            }
            joint_state_publisher_->publish(jointState);
        }
    }

    void PiperMujocoMsgHandler::joint_cmd_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
    {
        if (!sim_ || !sim_->d_ || !sim_->m_)
        {
            return;
        }

        for (size_t k = 0; k < msg->name.size(); k++)
        {
            const std::string &actuator_name = msg->name[k];
            int actuator_id = mj_name2id(sim_->m_, mjOBJ_ACTUATOR, actuator_name.c_str());
            int joint_id = mj_name2id(sim_->m_, mjOBJ_JOINT, actuator_name.c_str());

            if (actuator_id == -1)
            {
                RCLCPP_WARN(rclcpp::get_logger("MuJoCo"),
                            "Actuator '%s' not found in MuJoCo model.",
                            actuator_name.c_str());
                continue;
            }
            // Get the joint position and velocity directly from qpos and qvel
            // double joint_position = sim_->d_->qpos[sim_->m_->jnt_qposadr[joint_id]];
            // double joint_velocity = sim_->d_->qvel[sim_->m_->jnt_dofadr[joint_id]];
            //
            // // Calculate control signal
            // double position_error = msg->pos[k] - joint_position;
            // double velocity_error = msg->vel[k] - joint_velocity;

            // sim_->d_->ctrl[actuator_id] =
            //     msg->kp[k] * position_error + msg->kd[k] * velocity_error + msg->torque[k];

            sim_->d_->ctrl[actuator_id] = msg->position[k];

            // // Apply torque limits dynamically from the message
            // double torque_limit = msg->torque_limit[k];
            // sim_->d_->ctrl[actuator_id] =
            //     std::clamp(sim_->d_->ctrl[actuator_id], -torque_limit, torque_limit);
        }
    }

} // namespace Galileo
