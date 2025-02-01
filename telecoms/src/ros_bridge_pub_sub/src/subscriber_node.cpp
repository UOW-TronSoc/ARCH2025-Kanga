#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "custom_msgs/msg/custom_message.hpp"

#include <chrono>

using namespace std::chrono_literals;
using namespace std;

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("subscriber_node")
    {
        subscription_ = this->create_subscription<custom_msgs::msg::CustomMessage>(
            "example_topic", 10,
            std::bind(&SubscriberNode::receive_message, this, std::placeholders::_1));
    }

private:
    void receive_message(const custom_msgs::msg::CustomMessage::SharedPtr msg)
    {
        auto time = chrono::high_resolution_clock::now();
        int64_t duration_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();

        int64_t latency = duration_since_epoch - msg->epoch_time;
        RCLCPP_INFO(this->get_logger(), "Received: '%s', " "Latency: '%lf' ms", msg->data.c_str(), latency * 1e-6);
    }

    rclcpp::Subscription<custom_msgs::msg::CustomMessage>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
