#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/science_feedback.hpp"
#include "custom_msgs/msg/science_control.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using namespace std;

class MotorFeedbackNode : public rclcpp::Node
{
public:
    MotorFeedbackNode() : Node("science_feedback_node")
    {
        publisher_ = this->create_publisher<custom_msgs::msg::ScienceFeedback>("science_feedback", 10);
        timer_ = this->create_wall_timer(1000ms, std::bind(&MotorFeedbackNode::publish_message, this));
    }

private:
    void publish_message()                                                                          
    {
        auto message = custom_msgs::msg::ScienceFeedback();

        auto time = chrono::high_resolution_clock::now();
        int64_t duration_since_epoch = chrono::duration_cast<chrono::nanoseconds>(time.time_since_epoch()).count();

        // message.epoch_time = duration_since_epoch;
        message.water_percent = rand() % 101;
        message.temperature = rand() % 201;
        message.ilmenite_percent = rand() % 101;

        RCLCPP_INFO(this->get_logger(), "Publishing Science Feedback - Epoch Time: %ld", duration_since_epoch);
        publisher_->publish(message);
    }

    rclcpp::Publisher<custom_msgs::msg::ScienceFeedback>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorFeedbackNode>());
    rclcpp::shutdown();
    return 0;
}
