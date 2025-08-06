#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace std;

class SafetyController: public rclcpp::Node{
public:
    SafetyController(): Node("safety_controller"){
        auto_cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "autonomous/cmd_vel", 1, bind(&SafetyController::auto_cmd_vel_callback, this, placeholders::_1));
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy_teleop/joy", 1, bind(&SafetyController::joy_callback, this, placeholders::_1));

        auto_mode_cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "auto_mode/cmd_vel", 10);
    }

private:
    void auto_cmd_vel_callback(geometry_msgs::msg::TwistStamped::SharedPtr msg){
        auto_cmd_vel_ = msg;
    }

    void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg){
        float axis_value = msg->axes[4];

        if (axis_value == 1.0){
            if (!auto_cmd_vel_) {
                RCLCPP_WARN(this->get_logger(), "Haven't received Autonomous Command Yet");
                return;
            }
            
            auto_mode_cmd_vel_publisher_->publish(*auto_cmd_vel_);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr auto_cmd_vel_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr auto_mode_cmd_vel_publisher_;

    geometry_msgs::msg::TwistStamped::SharedPtr auto_cmd_vel_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    shared_ptr<SafetyController> node = make_shared<SafetyController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
