#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "custom_msgs/msg/drivetrain_control.hpp"
#include <chrono>

using namespace std::chrono_literals;
using namespace std;

class JoyListener : public rclcpp::Node
{
public:
  JoyListener()
  : Node("joy_listener")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoyListener::joy_callback, this, std::placeholders::_1)
    );

    drive_publisher_ = this->create_publisher<custom_msgs::msg::DrivetrainControl>("drive_commands", 10);  // <-- Added publisher
  
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::string axes_str = "[ ";
    for (auto axis : msg->axes) {
      axes_str += std::to_string(axis) + " ";
    }
    axes_str += "]";
    
    std::string buttons_str = "[ ";
    for (auto button : msg->buttons) {
      buttons_str += std::to_string(button) + " ";
    }
    buttons_str += "]";
    
    RCLCPP_INFO(this->get_logger(), "Received Joy: axes: %s, buttons: %s", axes_str.c_str(), buttons_str.c_str());


        // Create and publish drive command message
    auto drive_msg = custom_msgs::msg::DrivetrainControl();

    auto time = chrono::high_resolution_clock::now();
    int64_t duration_since_epoch = chrono::duration_cast<chrono::nanoseconds>(time.time_since_epoch()).count();

    drive_msg.epoch_time = duration_since_epoch;

    if (msg->axes.size() > 4) {
      // Assuming index 1 is the left joystick y axis and index 4 is the right joystick y axis
      drive_msg.lf_drive = static_cast<int8_t>(msg->axes[1] * 100);
      drive_msg.lb_drive = static_cast<int8_t>(msg->axes[1] * 100);
      drive_msg.rf_drive = static_cast<int8_t>(msg->axes[3] * 100);
      drive_msg.rb_drive = static_cast<int8_t>(msg->axes[3] * 100);
    } else {
      RCLCPP_WARN(this->get_logger(), "Joy axes size is smaller than expected.");
    }
    drive_publisher_->publish(drive_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<custom_msgs::msg::DrivetrainControl>::SharedPtr drive_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyListener>());
  rclcpp::shutdown();
  return 0;
}
