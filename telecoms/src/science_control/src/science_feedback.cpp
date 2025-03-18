#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/science_feedback.hpp"
#include "custom_msgs/msg/science_control.hpp"
#include <chrono>
#include <memory>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std;

class MotorFeedbackNode : public rclcpp::Node
{
public:
    MotorFeedbackNode() : Node("science_feedback_node")
    {
        publisher_ = this->create_publisher<custom_msgs::msg::ScienceFeedback>("science_feedback", 10);
        timer_ = this->create_wall_timer(1000ms, std::bind(&MotorFeedbackNode::publish_message, this));

        log_publisher_ = this->create_publisher<std_msgs::msg::String>("rover_logs", 10);
    }

private:
    void publish_message()                                                                          
    {
        auto message = custom_msgs::msg::ScienceFeedback();

        auto time = chrono::high_resolution_clock::now();
        int64_t duration_since_epoch = chrono::duration_cast<chrono::nanoseconds>(time.time_since_epoch()).count();

        // message.epoch_time = duration_since_epoch;
        message.water_percent = 11.0 + (rand() % 301)/100.0;
        message.temperature = 28.0 + (rand() % 201)/100.0;
        message.ilmenite_percent = 0;

        RCLCPP_INFO(this->get_logger(), "Publishing Science Feedback - Epoch Time: %lf", message.temperature);
        publisher_->publish(message);

        auto log_message = std_msgs::msg::String();

        log_message.data = "Publishing Science Feedback - Epoch Time: " + std::to_string(message.temperature) + " deg C, Water Percentage: " + std::to_string(message.water_percent);

        log_publisher_->publish(log_message);
    }

    rclcpp::Publisher<custom_msgs::msg::ScienceFeedback>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])    
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorFeedbackNode>());
    rclcpp::shutdown();
    return 0;
}
