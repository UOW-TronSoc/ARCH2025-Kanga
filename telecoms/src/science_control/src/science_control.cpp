#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/science_feedback.hpp"
#include "custom_msgs/msg/science_control.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using namespace std;

class ScienceControlNode : public rclcpp::Node
{
public:
    ScienceControlNode() : Node("science_feedback")
    {
        subscription_ = this->create_subscription<custom_msgs::msg::ScienceControl>(
            "science_control", 10,
            std::bind(&ScienceControlNode::receive_message, this, std::placeholders::_1));
    }

private:
    void receive_message(const custom_msgs::msg::ScienceControl::SharedPtr msg)
    {
        auto time = chrono::high_resolution_clock::now();
        int64_t duration_since_epoch = chrono::duration_cast<chrono::nanoseconds>(time.time_since_epoch()).count();

        // int64_t latency = duration_since_epoch - msg->epoch_time;
        RCLCPP_INFO(this->get_logger(), "Heat Moodule Status: ' %d ', Water Module Status: '%d', Ilmenite Module Status: '%d' Water Extraction Module Deployed: ' %d ' Sensor Module Deploy: ' %d", 
            msg->heat_status, msg->water_status, msg->ilmenite_status, msg->deploy_heat, msg->deploy_sensors);
    }

    rclcpp::Subscription<custom_msgs::msg::ScienceControl>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScienceControlNode>());
    rclcpp::shutdown();
    return 0;
}
