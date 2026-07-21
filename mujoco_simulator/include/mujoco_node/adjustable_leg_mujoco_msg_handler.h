//
// Created by lbt on 24-12-6.
//
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <nav_msgs/msg/odometry.hpp>
//
//
// #include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include "array_safety.h"
#include "simulate.h"
#include "custom_msgs/msg/sensor_msg.h"
#include "custom_msgs/msg/actuator_cmds.hpp"
#include "custom_msgs/msg/mujoco_msg.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <limits>
#include <atomic>
#include <unordered_map>
#include <vector>

using namespace rclcpp;

using namespace std::chrono_literals;

namespace ArcLab
{
    namespace mj = ::mujoco;
    namespace mju = ::mujoco::sample_util;

    class AdjustableLegMujocoMsgHandler : public rclcpp::Node
    {
    public:
        const std::string xml_file_path() { return xml_file_path_; }

        struct ActuatorCmds
        {
            double time = 0.0;
            std::vector<std::string> actuators_name;
            std::vector<float> kp;
            std::vector<float> pos;
            std::vector<float> kd;
            std::vector<float> vel;
            std::vector<float> torque;
        };

        AdjustableLegMujocoMsgHandler(mj::Simulate *sim);
        ~AdjustableLegMujocoMsgHandler();

    private:
        void publish_mujoco_callback();
        void imu_callback();
        void contact_callback();
        void joint_callback();
        void actuator_cmd_callback(const custom_msgs::msg::ActuatorCmds::SharedPtr msg);
        void diagnostic_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void arm_contract_diagnostic(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        mj::Simulate *sim_;
        std::vector<rclcpp::TimerBase::SharedPtr> timers_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        rclcpp::Publisher<custom_msgs::msg::MujocoMsg>::SharedPtr mujoco_msg_publisher_;
        rclcpp::Publisher<custom_msgs::msg::ActuatorCmds>::SharedPtr applied_actuator_cmd_publisher_;
        rclcpp::Subscription<custom_msgs::msg::ActuatorCmds>::SharedPtr actuator_cmd_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr diagnostic_joy_subscription_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr contract_diagnostic_arm_service_;
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
        std::string xml_file_path_;
        bool box_response_enabled_ = false;
        double box_response_max_speed_mps_ = 0.080;
        double box_target_lower_m_ = 0.0;
        double box_target_upper_m_ = 0.060;
        double box_response_last_sim_time_ = std::numeric_limits<double>::quiet_NaN();
        const mjModel *box_response_model_ = nullptr;
        std::unordered_map<int, double> box_applied_targets_;
        bool contract_diagnostic_enabled_ = false;
        double contract_platform_height_m_ = 0.30;
        std::atomic_bool contract_diagnostic_armed_{false};
        std::atomic_bool contract_diagnostic_restored_{false};
    };
} // Galileo
