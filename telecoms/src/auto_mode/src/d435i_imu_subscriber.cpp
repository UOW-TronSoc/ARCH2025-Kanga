#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class IMUSubscriber : public rclcpp::Node
{
public:
    IMUSubscriber() : Node("d435i_imu_subscriber")
    {
        // Subscribe to gyro topic
        gyro_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/camera/gyro/sample", 10,
            std::bind(&IMUSubscriber::gyro_callback, this, std::placeholders::_1));

        // Subscribe to accel topic
        accel_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/camera/accel/sample", 10,
            std::bind(&IMUSubscriber::accel_callback, this, std::placeholders::_1));
    }

private:
    void gyro_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Gyro Data: x=%.6f, y=%.6f, z=%.6f",
                    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    }

    void accel_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Accel Data: x=%.6f, y=%.6f, z=%.6f",
                    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_subscriber_;    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr accel_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
