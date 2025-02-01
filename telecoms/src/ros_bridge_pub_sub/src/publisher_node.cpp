#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "custom_msgs/msg/custom_message.hpp"
// #include "tutorial_interfaces/srv/add_three_ints.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using namespace std;


class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode() : Node("publisher_node")
    {
        publisher_ = this->create_publisher<custom_msgs::msg::CustomMessage>("example_topic", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&PublisherNode::publish_message, this));
    }

private:
    void publish_message()                                                                          
    {
        auto message = custom_msgs::msg::CustomMessage();

        auto time = chrono::high_resolution_clock::now();
        int64_t duration_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();

        message.epoch_time = duration_since_epoch;
        message.data = "Hello World!";
        message.flag = true;

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s',  " "%ld", message.data.c_str(), message.epoch_time);
        publisher_->publish(message);
    }

    rclcpp::Publisher<custom_msgs::msg::CustomMessage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}
