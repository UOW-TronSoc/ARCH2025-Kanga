#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "odrive_can/msg/control_message.hpp"

using namespace std::chrono_literals;

class ControlMessagePublisher : public rclcpp::Node
{
public:
    ControlMessagePublisher(float input_vel) : Node("control_message_publisher"), input_vel_(input_vel), max_publish_count_(5), current_count_(0)
    {
        // Create a publisher for the /odrive_axis0/control_message topic
        publisher_ = this->create_publisher<odrive_can::msg::ControlMessage>("/odrive_axis0/control_message", 10);

        // Create a timer to periodically publish the message
        timer_ = this->create_wall_timer(2ms, std::bind(&ControlMessagePublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        if (current_count_ >= max_publish_count_) {
            RCLCPP_INFO(this->get_logger(), "Reached maximum publish count (%d). Stopping timer.", max_publish_count_);
            timer_->cancel();  // Stop the timer
            return;
        }

        // Create and populate the message
        auto message = odrive_can::msg::ControlMessage();
        message.control_mode = 2;      // Control mode
        message.input_mode = 1;       // Input mode
        message.input_pos = 0.0;      // Input position
        message.input_vel = input_vel_; // Input velocity
        message.input_torque = 0.0;   // Input torque

        // Log the message being published
        RCLCPP_INFO(this->get_logger(), "Publishing ControlMessage #%d: control_mode=%d, input_mode=%d, input_pos=%.2f, input_vel=%.2f, input_torque=%.2f",
                    current_count_ + 1, message.control_mode, message.input_mode, message.input_pos, message.input_vel, message.input_torque);

        // Publish the message
        publisher_->publish(message);

        // Increment the publish count
        current_count_++;
    }

    rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float input_vel_;  // Input velocity
    int max_publish_count_;  // Maximum number of times to publish
    int current_count_;  // Counter to track the number of messages published
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Ensure the user provides the input_vel argument
    if (argc != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: control_message_publisher <input_vel>");
        return 1;
    }

    try {
        // Convert input_vel from command-line argument to float
        float input_vel = std::stof(argv[1]);

        // Create the node
        auto node = std::make_shared<ControlMessagePublisher>(input_vel);

        // Spin the node to keep it alive
        rclcpp::spin(node);
    } catch (const std::invalid_argument &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid input_vel: %s. Must be a valid number.", argv[1]);
        return 1;
    } catch (const std::out_of_range &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Input_vel out of range: %s.", argv[1]);
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
