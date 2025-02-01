#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/drivetrain_feedback.hpp"
#include "custom_msgs/msg/drivetrain_control.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using namespace std;

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode() : Node("drivetrain_feedback_node")
    {
        publisher_ = this->create_publisher<custom_msgs::msg::DrivetrainFeedback>("drivetrain_feedback", 10);

        subscription_ = this->create_subscription<custom_msgs::msg::DrivetrainControl>(
            "drive_commands", 10,
            std::bind(&MotorControlNode::receive_message, this, std::placeholders::_1));

        
    }

private:
    void publish_message()                                                                          
    {
        auto message = custom_msgs::msg::DrivetrainFeedback();

        auto time = chrono::high_resolution_clock::now();
        int64_t duration_since_epoch = chrono::duration_cast<chrono::nanoseconds>(time.time_since_epoch()).count();

        message.epoch_time = duration_since_epoch;
        message.wheel_position = {0.0, 1.0, 2.0, 3.0};
        message.wheel_velocity = {4.0, 5.0, 6.0, 7.0};
        message.wheel_torque = {8.0, 9.0, 10.0, 11.0};

        RCLCPP_INFO(this->get_logger(), "Publishing DrivetrainFeedback - Epoch Time: %ld", message.epoch_time);
        publisher_->publish(message);
    }

    void receive_message(const custom_msgs::msg::DrivetrainControl::SharedPtr msg)
    {
        auto time = chrono::high_resolution_clock::now();
        int64_t duration_since_epoch = chrono::duration_cast<chrono::nanoseconds>(time.time_since_epoch()).count();

        int64_t latency = duration_since_epoch - msg->epoch_time;
        RCLCPP_INFO(this->get_logger(), "Left Drive: '%d', Right Drive: '%d', Latency: '%.3f' ms",
            msg->left_drive, msg->right_drive, latency * 1e-6);
    }

    rclcpp::Subscription<custom_msgs::msg::DrivetrainControl>::SharedPtr subscription_;
    rclcpp::Publisher<custom_msgs::msg::DrivetrainFeedback>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}
