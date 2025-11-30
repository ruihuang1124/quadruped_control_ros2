#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/user_cmds.hpp>
#include <control_input_msgs/msg/inputs.hpp>
// #include <control_input_msgs/msg/pose_cmd_inputs.hpp>
#include <std_msgs/msg/string.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;

class CmdMapping : public rclcpp::Node
{
public:
    CmdMapping(std::string name) : Node(name)
    {

        keyboard_subscriptor = this->create_subscription<std_msgs::msg::String>(
            "keyboard_input", 10, std::bind(&CmdMapping::cmdMappingCallback, this, _1));
        cmd_publisher = this->create_publisher<custom_msgs::msg::UserCmds>("user_cmd", 10);
        control_input_publisher_ = this->create_publisher<control_input_msgs::msg::Inputs>("/control_input", 10);
        // pose_control_input_publisher_ = this->create_publisher<control_input_msgs::msg::PoseCmdInputs>("/pose_control_input", 10);

        switch_controller_client = this->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");

        initUserCmd();
        initControlInputCmd();
        // initPoseControlInputCmd();

        RCLCPP_INFO(this->get_logger(), "Command mapping node started in 50ms.");
    }

private:
    custom_msgs::msg::UserCmds user_cmd_;
    control_input_msgs::msg::Inputs control_input_cmd_;
    // control_input_msgs::msg::PoseCmdInputs pose_control_input_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriptor;
    rclcpp::Publisher<custom_msgs::msg::UserCmds>::SharedPtr cmd_publisher;
    rclcpp::Publisher<control_input_msgs::msg::Inputs>::SharedPtr control_input_publisher_;
    // rclcpp::Publisher<control_input_msgs::msg::PoseCmdInputs>::SharedPtr pose_control_input_publisher_;

    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client;

    void cmdMappingCallback(const std_msgs::msg::String keyboard_input)
    {
        // initControlInputCmd();
        // initPoseControlInputCmd();
        switch (keyboard_input.data[0])
        {
        
        // case 'w':
        // case 'W':
        //     user_cmd_.linear_x_input += 0.1;
        //     break;
        // case 's':
        // case 'S':
        //     user_cmd_.linear_x_input = 0.0;
        //     user_cmd_.linear_y_input = 0.0;
        //     user_cmd_.angular_y_input = 0.0;
        //     user_cmd_.angular_z_input = 0.0;
        //     break;
        // case 'a':
        // case 'A':
        //     user_cmd_.linear_y_input += 0.1;
        //     break;
        // case 'd':
        // case 'D':
        //     user_cmd_.linear_y_input -= 0.1;
        //     break;
        case 'i':
        case 'I':
            user_cmd_.angular_y_input += 0.05;
            break;
        case 'k':
        case 'K':
            user_cmd_.angular_y_input -= 0.05;
            break;
        case 'j':
        case 'J':
            user_cmd_.angular_z_input += 0.1;
            break;
        case 'l':
        case 'L':
            user_cmd_.angular_z_input -= 0.1;
            break;
        case 'b':
        case 'B':
            user_cmd_.linear_x_input -= 0.1;
            break;
        case 'r':
        case 'R':
            user_cmd_.height_ratio += 0.2;
            break;
        case 'f':
        case 'F':
            user_cmd_.height_ratio -= 0.2;
            break;
        // case 'W':
        //     pose_control_input_cmd_.pos_x += 0.1;
        //     break;
        // case 's':
        // case 'S':
        //     pose_control_input_cmd_.pos_x -= 0.1;
        //     break;
        // case 'a':
        // case 'A':
        //     pose_control_input_cmd_.pos_y += 0.1;
        //     break;
        // case 'd':
        // case 'D':
        //     pose_control_input_cmd_.pos_y -= 0.1;
            // break;
        case '0':
        case ')':
            initControlInputCmd();
            break;
        case 'w':
        case 'W':
            control_input_cmd_.lx += 0.25;
            break;
        case 's':
        case 'S':
            control_input_cmd_.lx -= 0.25;
            break;
        case 'a':
        case 'A':
            control_input_cmd_.ly += 0.25;
            break;
        case 'd':
        case 'D':
            control_input_cmd_.ly -= 0.25;
            break;
        case '-':
        case '_':
            control_input_cmd_.rx -= 0.25;
            break;
        case '=':
        case '+':
            control_input_cmd_.rx += 0.25;
            break;
        case '1':
            user_cmd_.gait_name = "stance";
            control_input_cmd_.command = 1;
            break;
        case '2':
            user_cmd_.gait_name = "trot";
            control_input_cmd_.command = 2;
            break;
        case '3':
            user_cmd_.gait_name = "standing_trot";
            control_input_cmd_.command = 3;
            break;
        case '4':
            user_cmd_.gait_name = "flying_trot";
            control_input_cmd_.command = 4;
            break;
        case '5':
            user_cmd_.gait_name = "standing_pace";
            control_input_cmd_.command = 5;
            break;
        case '6':
            user_cmd_.gait_name = "dynamic_walk";
            control_input_cmd_.command = 6;
            break;
        case '7':
            user_cmd_.gait_name = "bound";
            break;
        case ' ':
            user_cmd_.passive_enable = true;
            break;
        case '9':
            handleSwitchController();
            break;
        }

        user_cmd_.height_ratio = std::clamp(user_cmd_.height_ratio, 0.0, 1.0);

        cmd_publisher->publish(user_cmd_);
        control_input_publisher_->publish(control_input_cmd_);
        // pose_control_input_publisher_->publish(pose_control_input_cmd_);
    }

    void initUserCmd()
    {
        user_cmd_.linear_x_input = 0.0;
        user_cmd_.linear_y_input = 0.0;
        user_cmd_.angular_y_input = 0.0;
        user_cmd_.angular_z_input = 0.0;

        user_cmd_.height_ratio = 1.0;
        user_cmd_.gait_name = "stance";
        user_cmd_.passive_enable = false;
    }

    void initControlInputCmd()
    {
        control_input_cmd_.command = 0;
        control_input_cmd_.lx = 0.0;
        control_input_cmd_.ly = 0.0;
        control_input_cmd_.rx = 0.0;
        control_input_cmd_.ry = 0.0;
    }

    // void initPoseControlInputCmd() {
    //     pose_control_input_cmd_.pos_x = 0.55;
    //     pose_control_input_cmd_.pos_y = 0.0;
    //     pose_control_input_cmd_.pos_z = 0.3;
    //     pose_control_input_cmd_.pos_roll = 0.0;
    //     pose_control_input_cmd_.pos_pitch = 3.14;
    //     pose_control_input_cmd_.pos_yaw = 0.0;
    // }

    void handleSwitchController()
    {
        if (!switch_controller_client->wait_for_service(1s))
        {
            RCLCPP_ERROR(this->get_logger(), "Service not available after waiting");
            return;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers.push_back("ocs2_quadruped_controller");

        switch_controller_client->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Request to activate the controller: ocs2_quadruped_controller");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdMapping>("cmd_mapping");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}