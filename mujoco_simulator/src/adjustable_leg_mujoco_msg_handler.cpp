#include "adjustable_leg_mujoco_msg_handler.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "sensor_msgs/image_encodings.hpp"

namespace ArcLab
{
    AdjustableLegMujocoMsgHandler::AdjustableLegMujocoMsgHandler(mj::Simulate *sim)
        : Node("AdjustableLegMujocoMsgHandler", rclcpp::NodeOptions().use_intra_process_comms(true)),
          sim_(sim)
    {
        this->declare_parameter<std::string>("xml_file_path", "path/to/your/xml");
        this->declare_parameter<bool>("box_response_enabled", false);
        // Both independently recorded 0707 trials fit the same q_des -> q_actual
        // slew limit (0.076 and 0.078 m/s).  Use the rounded common value here.
        // Do not derive this value from qd_prismatic: that telemetry is empirically
        // under-scaled by about 18--19x relative to d(q_prismatic)/dt.
        this->declare_parameter<double>("box_response_max_speed_mps", 0.080);
        this->declare_parameter<double>("box_target_lower_m", 0.0);
        this->declare_parameter<double>("box_target_upper_m", 0.060);
        this->declare_parameter<bool>("highstep_contract_diagnostic_enabled", false);
        this->declare_parameter<double>("highstep_contract_platform_height_m", 0.30);
        this->get_parameter("xml_file_path", xml_file_path_);
        this->get_parameter("box_response_enabled", box_response_enabled_);
        this->get_parameter("box_response_max_speed_mps", box_response_max_speed_mps_);
        this->get_parameter("box_target_lower_m", box_target_lower_m_);
        this->get_parameter("box_target_upper_m", box_target_upper_m_);
        this->get_parameter("highstep_contract_diagnostic_enabled", contract_diagnostic_enabled_);
        this->get_parameter("highstep_contract_platform_height_m", contract_platform_height_m_);
        if (!(contract_platform_height_m_ > 0.0 && contract_platform_height_m_ < 0.60))
        {
            throw std::invalid_argument("highstep_contract_platform_height_m must be in (0, 0.60)");
        }

        if (box_response_max_speed_mps_ <= 0.0)
        {
            throw std::invalid_argument("box_response_max_speed_mps must be positive");
        }
        if (box_target_lower_m_ >= box_target_upper_m_)
        {
            throw std::invalid_argument("box_target_lower_m must be smaller than box_target_upper_m");
        }
        std::cout << "xml file path:" << xml_file_path_ << std::endl;

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", qos);
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
        mujoco_msg_publisher_ = this->create_publisher<custom_msgs::msg::MujocoMsg>("mujoco_msg", qos);
        applied_actuator_cmd_publisher_ = this->create_publisher<custom_msgs::msg::ActuatorCmds>(
            "mujoco_applied_actuators_cmds", qos);

        timers_.emplace_back(this->create_wall_timer(1ms, std::bind(&AdjustableLegMujocoMsgHandler::publish_mujoco_callback, this)));

        actuator_cmd_subscription_ = this->create_subscription<custom_msgs::msg::ActuatorCmds>(
            "actuators_cmds",
            qos,
            std::bind(&AdjustableLegMujocoMsgHandler::actuator_cmd_callback, this, std::placeholders::_1));

        if (contract_diagnostic_enabled_)
        {
            diagnostic_joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "/joy", 10,
                std::bind(&AdjustableLegMujocoMsgHandler::diagnostic_joy_callback, this, std::placeholders::_1));
            contract_diagnostic_arm_service_ = this->create_service<std_srvs::srv::Trigger>(
                "/highstep_contract_arm",
                std::bind(
                    &AdjustableLegMujocoMsgHandler::arm_contract_diagnostic,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
            RCLCPP_WARN(
                this->get_logger(),
                "Highstep contract diagnostic is enabled. It is read-only with respect to policy weights, "
                "but the next armed full-stick event will reset MuJoCo to the approved B300 start state.");
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Start AdjustableLegMujocoMsgHandler (box response: %s, max target speed: %.6f m/s, target range: [%.3f, %.3f] m)",
            box_response_enabled_ ? "enabled" : "disabled",
            box_response_max_speed_mps_,
            box_target_lower_m_,
            box_target_upper_m_);
        sim_->uiloadrequest.fetch_add(1);
    }

    AdjustableLegMujocoMsgHandler::~AdjustableLegMujocoMsgHandler()
    {
        RCLCPP_INFO(this->get_logger(), "close node ...");
    }

    void AdjustableLegMujocoMsgHandler::arm_contract_diagnostic(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!contract_diagnostic_enabled_)
        {
            response->success = false;
            response->message = "highstep contract diagnostic is disabled";
            return;
        }
        contract_diagnostic_armed_ = true;
        contract_diagnostic_restored_ = false;
        response->success = true;
        response->message = "armed: next full-forward /joy event restores the approved B300 state";
    }

    void AdjustableLegMujocoMsgHandler::diagnostic_joy_callback(
        const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!contract_diagnostic_enabled_ || !contract_diagnostic_armed_
            || contract_diagnostic_restored_ || msg->axes.size() <= 1 || msg->axes[1] < 0.99F
            || !sim_ || !sim_->m_ || !sim_->d_)
        {
            return;
        }

        const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
        const int key_id = mj_name2id(sim_->m_, mjOBJ_KEY, "highstep_b300_contract_start");
        const int platform_id = mj_name2id(sim_->m_, mjOBJ_GEOM, "highstep_box_L10_h0350");
        if (key_id < 0 || platform_id < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Cannot restore highstep contract: keyframe=%d platform=%d", key_id, platform_id);
            contract_diagnostic_armed_ = false;
            return;
        }

        // Environment setup may set the physical platform height. The policy
        // adapter never receives this value, geom id, or any MuJoCo pointer.
        sim_->m_->geom_pos[3 * platform_id + 2] = 0.5 * contract_platform_height_m_;
        sim_->m_->geom_size[3 * platform_id + 2] = 0.5 * contract_platform_height_m_;
        mj_resetDataKeyframe(sim_->m_, sim_->d_, key_id);
        mj_forward(sim_->m_, sim_->d_);
        box_applied_targets_.clear();
        box_response_last_sim_time_ = std::numeric_limits<double>::quiet_NaN();
        contract_diagnostic_restored_ = true;
        contract_diagnostic_armed_ = false;
        RCLCPP_WARN(
            this->get_logger(),
            "[HIGHSTEP_CONTRACT_RESET] restored approved q/dq, base=(0.65,0,0.44), "
            "edge_gap=0.55, physical_platform_height=%.3f on full-stick trigger; adapter fake scan remains frozen",
            contract_platform_height_m_);
    }

    void AdjustableLegMujocoMsgHandler::publish_mujoco_callback()
    {
        if (sim_->d_ != nullptr)
        {
            const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
            imu_callback();
            joint_callback();
            contact_callback();
        }
    }

    void AdjustableLegMujocoMsgHandler::imu_callback()
    {
        auto message = sensor_msgs::msg::Imu();
        message.header.frame_id = &sim_->m_->names[0];
        message.header.stamp = rclcpp::Clock().now();

        for (int i = 0; i < sim_->m_->nsensor; i++)
        {
            if (sim_->m_->sensor_type[i] == mjtSensor::mjSENS_ACCELEROMETER)
            {
                message.linear_acceleration.x = sim_->d_->sensordata[sim_->m_->sensor_adr[i]];
                message.linear_acceleration.y = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 1];
                message.linear_acceleration.z = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 2];
            }
            else if (sim_->m_->sensor_type[i] == mjtSensor::mjSENS_FRAMEQUAT)
            {
                message.orientation.w = sim_->d_->sensordata[sim_->m_->sensor_adr[i]];
                message.orientation.x = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 1];
                message.orientation.y = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 2];
                message.orientation.z = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 3];
            }
            else if (sim_->m_->sensor_type[i] == mjtSensor::mjSENS_GYRO)
            {
                message.angular_velocity.x = sim_->d_->sensordata[sim_->m_->sensor_adr[i]];
                message.angular_velocity.y = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 1];
                message.angular_velocity.z = sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 2];
            }
        }

        imu_publisher_->publish(message);
    }

    void AdjustableLegMujocoMsgHandler::contact_callback()
    {
        auto msg = custom_msgs::msg::MujocoMsg();
        std::vector<std::string> foot_geom_names = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"}; // TODO: 改成参数传递

        std::vector<int> foot_geom_ids;
        for (const auto &foot_name : foot_geom_names)
        {
            int geom_id = mj_name2id(sim_->m_, mjOBJ_GEOM, foot_name.c_str());
            if (geom_id == -1)
            {
                std::cerr << "Geometry " << foot_name << " not found in the model!" << std::endl;
                return;
            }
            foot_geom_ids.push_back(geom_id);
        }
        int ground_geom_id = mj_name2id(sim_->m_, mjOBJ_GEOM, "floor");

        // 初始化接触状态数组，默认值为 0（未接触）
        int contact_state[4] = {0, 0, 0, 0};
        // 初始化12维接触力向量,每条腿3维力
        std::vector<double> contact_forces(12, 0.0);

        for (int i = 0; i < sim_->d_->ncon; ++i)
        {
            const mjContact &contact = sim_->d_->contact[i];

            // 遍历所有足端
            for (size_t foot_idx = 0; foot_idx < foot_geom_ids.size(); ++foot_idx)
            {
                int foot_geom_id = foot_geom_ids[foot_idx];

                // 检查是否是足端与地面的接触
                if ((contact.geom1 == foot_geom_id && contact.geom2 == ground_geom_id) ||
                    (contact.geom1 == ground_geom_id && contact.geom2 == foot_geom_id))
                {
                    // 如果发生接触，标记对应的接触状态为 1
                    contact_state[foot_idx] = 1;

                    // 提取接触力
                    mjtNum force[6]; // 包含法向力和摩擦力的 6D 力
                    mj_contactForce(sim_->m_, sim_->d_, i, force);

                    // 将接触力存入对应腿的位置(每条腿3维力)
                    contact_forces[foot_idx * 3] = force[0];
                    contact_forces[foot_idx * 3 + 1] = force[1];
                    contact_forces[foot_idx * 3 + 2] = force[2];
                }
            }
        }

        // 将接触力和接触状态填入消息
        msg.ground_reaction_force = contact_forces;
        for (int i = 0; i < 4; i++)
        {
            msg.contact_state[i] = contact_state[i];
        }

        mujoco_msg_publisher_->publish(msg);
    }

    void AdjustableLegMujocoMsgHandler::joint_callback()
    {
        if (sim_->d_ != nullptr)
        {
            sensor_msgs::msg::JointState jointState;
            jointState.header.frame_id = &sim_->m_->names[0];
            jointState.header.stamp = rclcpp::Clock().now();
            for (int i = 0; i < sim_->m_->njnt; i++)
            {
                // if (sim_->m_->jnt_type[i] == mjtJoint::mjJNT_HINGE)
                if (sim_->m_->jnt_type[i] == mjtJoint::mjJNT_HINGE || sim_->m_->jnt_type[i] == mjtJoint::mjJNT_SLIDE)
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

    void AdjustableLegMujocoMsgHandler::actuator_cmd_callback(
        const custom_msgs::msg::ActuatorCmds::SharedPtr msg)
    {
        if (!sim_ || !sim_->d_ || !sim_->m_)
        {
            return;
        }

        // Keep the legacy control target and PD equation unchanged when the calibrated response is disabled.
        // When enabled, the lock makes the response state and MuJoCo state share one simulation-time
        // snapshot.  This avoids advancing the four internal box targets while MuJoCo is resetting.
        std::unique_lock<std::recursive_mutex> sim_lock(sim_->mtx, std::defer_lock);
        if (box_response_enabled_)
        {
            sim_lock.lock();
        }

        custom_msgs::msg::ActuatorCmds applied_msg;
        double response_dt = 0.0;
        if (box_response_enabled_)
        {
            applied_msg = *msg;
            const double sim_time = sim_->d_->time;
            const bool model_changed = box_response_model_ != sim_->m_;
            const bool time_reset = !std::isfinite(box_response_last_sim_time_) ||
                                    sim_time + 1.0e-12 < box_response_last_sim_time_;
            if (model_changed || time_reset)
            {
                box_applied_targets_.clear();
                box_response_model_ = sim_->m_;
            }
            else
            {
                response_dt = std::max(0.0, sim_time - box_response_last_sim_time_);
            }
            box_response_last_sim_time_ = sim_time;
        }

        for (size_t k = 0; k < msg->actuators_name.size(); k++)
        {
            const std::string &actuator_name = msg->actuators_name[k];
            int actuator_id = mj_name2id(sim_->m_, mjOBJ_ACTUATOR, actuator_name.c_str());
            int joint_id = mj_name2id(sim_->m_, mjOBJ_JOINT, actuator_name.c_str());

            if (actuator_id == -1 || joint_id == -1)
            {
                RCLCPP_WARN(rclcpp::get_logger("MuJoCo"),
                            "Actuator or joint '%s' not found in MuJoCo model.",
                            actuator_name.c_str());
                continue;
            }
            // Get the joint position and velocity directly from qpos and qvel
            double joint_position = sim_->d_->qpos[sim_->m_->jnt_qposadr[joint_id]];
            double joint_velocity = sim_->d_->qvel[sim_->m_->jnt_dofadr[joint_id]];

            double applied_position_target = msg->pos[k];
            if (box_response_enabled_ && actuator_name.ends_with("_box_joint"))
            {
                const double requested_target = std::clamp(
                    static_cast<double>(msg->pos[k]), box_target_lower_m_, box_target_upper_m_);
                auto [target_it, inserted] = box_applied_targets_.try_emplace(
                    actuator_id,
                    std::clamp(joint_position, box_target_lower_m_, box_target_upper_m_));

                // Passive deliberately publishes position=0 with kp=0.  The
                // response model must follow the unloaded mechanism in that
                // state instead of rate-limiting its private target toward
                // zero.  Otherwise enabling FixedDown rebases a large stale
                // error through kp=4000 even though the controller's first
                // requested target is the current measured position.
                if (msg->kp[k] <= 0.0F)
                {
                    target_it->second = std::clamp(
                        joint_position, box_target_lower_m_, box_target_upper_m_);
                }
                else if (!inserted && response_dt > 0.0)
                {
                    const double max_delta = box_response_max_speed_mps_ * response_dt;
                    const double target_delta = std::clamp(
                        requested_target - target_it->second, -max_delta, max_delta);
                    target_it->second = std::clamp(
                        target_it->second + target_delta, box_target_lower_m_, box_target_upper_m_);
                }
                applied_position_target = target_it->second;
                applied_msg.pos[k] = applied_position_target;
            }

            // Keep the existing MuJoCo PD gains and effort command.  Only the box position
            // reference is rate-limited to the measured loaded mechanism speed.
            double position_error = applied_position_target - joint_position;
            double velocity_error = msg->vel[k] - joint_velocity;

            sim_->d_->ctrl[actuator_id] =
                msg->kp[k] * position_error + msg->kd[k] * velocity_error + msg->torque[k];

            // // Apply torque limits dynamically from the message
            // double torque_limit = msg->torque_limit[k];
            // sim_->d_->ctrl[actuator_id] =
            //     std::clamp(sim_->d_->ctrl[actuator_id], -torque_limit, torque_limit);
        }

        if (box_response_enabled_)
        {
            sim_lock.unlock();
            applied_actuator_cmd_publisher_->publish(applied_msg);
        }
    }

} // namespace Galileo
