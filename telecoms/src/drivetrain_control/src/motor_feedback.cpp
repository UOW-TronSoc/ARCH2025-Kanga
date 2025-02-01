#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/drivetrain_feedback.hpp"
#include "custom_msgs/msg/drivetrain_control.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using namespace std;

class MotorFeedbackNode : public rclcpp::Node
{
public:
    MotorFeedbackNode() : Node("drivetrain_feedback_node")
    {
        publisher_ = this->create_publisher<custom_msgs::msg::DrivetrainFeedback>("drivetrain_feedback", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&MotorFeedbackNode::publish_message, this));

       
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

    rclcpp::Publisher<custom_msgs::msg::DrivetrainFeedback>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorFeedbackNode>());
    rclcpp::shutdown();
    return 0;
}
