#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <iostream>

class OdomSubscriber : public rclcpp::Node {
public:
    OdomSubscriber() : Node("odom_listener") {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&OdomSubscriber::odom_callback, this, std::placeholders::_1)
        );
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received Odom Data:");

        // Extract Position
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
        RCLCPP_INFO(this->get_logger(), "Position -> x: %.4f, y: %.4f, z: %.4f", x, y, z);
            
        // Extract Quaternion Orientation
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        // Convert Quaternion to Euler Angles (Roll, Pitch, Yaw)
        tf2::Quaternion quat(qx, qy, qz, qw);
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "Orientation (Euler) -> Roll: %.4f, Pitch: %.4f, Yaw: %.4f", roll, pitch, yaw);

        // Extract Linear Velocity
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        double vz = msg->twist.twist.linear.z;
        RCLCPP_INFO(this->get_logger(), "Linear Velocity -> x: %.4f, y: %.4f, z: %.4f", vx, vy, vz);

        // Extract Angular Velocity
        double wx = msg->twist.twist.angular.x;
        double wy = msg->twist.twist.angular.y;
        double wz = msg->twist.twist.angular.z;
        RCLCPP_INFO(this->get_logger(), "Angular Velocity -> x: %.4f, y: %.4f, z: %.4f", wx, wy, wz);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSubscriber>());
    rclcpp::shutdown();
    return 0;
}
