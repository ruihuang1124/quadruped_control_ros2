#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/user_cmds.hpp>
#include <control_input_msgs/msg/inputs.hpp>
#include <control_input_msgs/msg/pose_cmd_inputs.hpp>
#include <std_msgs/msg/string.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;

class CmdMapping : public rclcpp::Node {
public:
    CmdMapping(std::string name) : Node(name) {
        keyboard_subscriptor = this->create_subscription<std_msgs::msg::String>(
            "keyboard_input", 10, std::bind(&CmdMapping::cmdMappingCallback, this, _1));
        cmd_publisher = this->create_publisher<custom_msgs::msg::UserCmds>("user_cmd", 10);
        control_input_publisher_ = this->create_publisher<control_input_msgs::msg::Inputs>("/control_input", 10);
        pose_control_input_publisher_ = this->create_publisher<control_input_msgs::msg::PoseCmdInputs>(
            "/pose_control_input", 10);
        switch_controller_client = this->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");

        initUserCmd();
        initControlInputCmd();
        initPoseControlInputCmd();

        RCLCPP_INFO(this->get_logger(), "Command mapping node started in 50ms.");
    }

private:
    custom_msgs::msg::UserCmds user_cmd_;
    control_input_msgs::msg::Inputs control_input_cmd_;
    control_input_msgs::msg::PoseCmdInputs pose_control_input_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriptor;
    rclcpp::Publisher<custom_msgs::msg::UserCmds>::SharedPtr cmd_publisher;
    rclcpp::Publisher<control_input_msgs::msg::Inputs>::SharedPtr control_input_publisher_;
    rclcpp::Publisher<control_input_msgs::msg::PoseCmdInputs>::SharedPtr pose_control_input_publisher_;

    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client;

    void cmdMappingCallback(const std_msgs::msg::String keyboard_input) {
        switch (keyboard_input.data[0]) {
            // Position control
            case 'j':
                pose_control_input_cmd_.pos_y += 0.05;
                break;
            case 'l':
                pose_control_input_cmd_.pos_y -= 0.05;
                break;
            case 'i':
                pose_control_input_cmd_.pos_x += 0.05;
                break;
            case 'k':
                pose_control_input_cmd_.pos_x -= 0.05;
                break;
            case 'u':
                pose_control_input_cmd_.pos_z += 0.05;
                break;
            case 'o':
                pose_control_input_cmd_.pos_z -= 0.05;
                break;

            // Orientation Control
            case '7':
                pose_control_input_cmd_.pos_roll += 0.05;
                break;
            case '4':
                pose_control_input_cmd_.pos_roll -= 0.05;
                break;
            case '8':
                pose_control_input_cmd_.pos_pitch += 0.05;
                break;
            case '5':
                pose_control_input_cmd_.pos_pitch -= 0.05;
                break;
            case '9':
                pose_control_input_cmd_.pos_yaw += 0.05;
                break;
            case '6':
                pose_control_input_cmd_.pos_yaw -= 0.05;
                break;

            case 'r': // reset
                pose_control_input_cmd_.pos_x = 0.1;
                pose_control_input_cmd_.pos_y = 0.0;
                pose_control_input_cmd_.pos_z = 0.30;
                pose_control_input_cmd_.pos_roll = 0.0;
                pose_control_input_cmd_.pos_pitch = 3.14;
                pose_control_input_cmd_.pos_yaw = 0.0;
                break;
                // Fingers
            // case '1': // catch
            //     pose_control_input_cmd_.L_finger_pos = 0.017;
            //     pose_control_input_cmd_.R_finger_pos = -0.017;
            //     break;
            // case '2': // release
            //     pose_control_input_cmd_.L_finger_pos = 0.035;
            //     pose_control_input_cmd_.R_finger_pos = -0.035;
            //     break;
        }
        pose_control_input_publisher_->publish(pose_control_input_cmd_);
    }

    void initUserCmd() {
        user_cmd_.linear_x_input = 0.0;
        user_cmd_.linear_y_input = 0.0;
        user_cmd_.angular_y_input = 0.0;
        user_cmd_.angular_z_input = 0.0;

        user_cmd_.height_ratio = 1.0;
        user_cmd_.gait_name = "stance";
        user_cmd_.passive_enable = false;
    }

    void initControlInputCmd() {
        control_input_cmd_.command = 0;
        control_input_cmd_.lx = 0.0;
        control_input_cmd_.ly = 0.0;
        control_input_cmd_.rx = 0.0;
        control_input_cmd_.ry = 0.0;
    }

    void initPoseControlInputCmd() {
        pose_control_input_cmd_.pos_x = 0.1;
        pose_control_input_cmd_.pos_y = 0.0;
        pose_control_input_cmd_.pos_z = 0.3;
        pose_control_input_cmd_.pos_roll = 0.0;
        pose_control_input_cmd_.pos_pitch = 3.14;
        pose_control_input_cmd_.pos_yaw = 0.0;
    }

    void handleSwitchController() {
        if (!switch_controller_client->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Service not available after waiting");
            return;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers.push_back("ocs2_quadruped_controller");

        switch_controller_client->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Request to activate the controller: ocs2_quadruped_controller");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdMapping>("cmd_mapping");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
