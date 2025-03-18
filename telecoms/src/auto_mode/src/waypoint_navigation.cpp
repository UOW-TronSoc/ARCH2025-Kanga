#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <custom_msgs/msg/drivetrain_control.hpp>
#include <cmath>
#include <vector>
#include <tuple>
#include <algorithm>

class WaypointNavigator : public rclcpp::Node {
public:
    WaypointNavigator() : Node("waypoint_navigator"), current_waypoint_index_(0) {
        // Define waypoints (x, y, theta) in meters and radians
        waypoints_ = {
            {2.0, 0.0, 0.0},     // Move 2 meters forward
            {2.0, 0.0, 1.57},    // Turn left 90 degrees at the same position
            {2.0, 1.0, 1.57}     // Move forward 1 meter to (2,1)
        };

        // Initialize robot position
        x_c_ = 0.0;
        y_c_ = 0.0;
        theta_r_ = 0.0;
        x_r_ = 0.0;
        y_r_ = 0.0;

        // Define tolerances
        distance_tolerance_ = 0.05;  // Stop within 5 cm of target
        angle_tolerance_ = 0.1;      // Allow 0.1 rad (~5°) error

        // Camera offset (meters) for localization adjustment
        camOffsetX = 0.32;
        camOffsetY = 0.22;

        // Proportional gains for control
        K_linear_ = 35.0;  // Speed factor
        K_angular_ = 45.0; // Turning factor
        K_final_turn_ = 30.0;  // Final orientation correction

        // Subscribe to odometry for position updates
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(), std::bind(&WaypointNavigator::odom_callback, this, std::placeholders::_1));

        // Publisher for drivetrain commands
        drive_pub_ = this->create_publisher<custom_msgs::msg::DrivetrainControl>("drive_commands", 10);

        // Timer for the control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&WaypointNavigator::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "🚀 Waypoint Navigator Initialized!");
    }

private:
    struct Waypoint {
        double x;
        double y;
        double theta;
    };

    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;

    double x_c_, y_c_, theta_r_, x_r_, y_r_;
    double distance_tolerance_;
    double angle_tolerance_;

    double camOffsetX, camOffsetY;
    double K_linear_, K_angular_, K_final_turn_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<custom_msgs::msg::DrivetrainControl>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Extract robot orientation (quaternion → yaw)
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        theta_r_ = yaw + M_PI / 2; // Adjust for robot reference frame

        // Extract position (adjusted for camera offset)
        x_c_ = -msg->pose.pose.position.y + camOffsetX;
        y_c_ = msg->pose.pose.position.x - camOffsetY;

        // Convert to robot-centered coordinates
        x_r_ = x_c_ - (camOffsetX * std::cos(theta_r_) + camOffsetY * std::sin(theta_r_));
        y_r_ = -y_c_ + (camOffsetX * std::sin(theta_r_) - camOffsetY * std::cos(theta_r_));
    }

    void control_loop() {
        if (current_waypoint_index_ >= waypoints_.size()) {
            stop_robot();
            return;
        }

        Waypoint wp = waypoints_[current_waypoint_index_];

        double dx = wp.x - x_r_;
        double dy = wp.y - y_r_;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_heading = std::atan2(dy, dx);
        double heading_error = normalize_angle(target_heading - theta_r_);

        RCLCPP_INFO(this->get_logger(), 
            "📍 Rover Pos: (%.2f, %.2f) | Target: (%.2f, %.2f) | Dist: %.2f | Heading Error: %.2f rad (%.1f°)", 
            x_r_, y_r_, wp.x, wp.y, distance, heading_error, heading_error * (180.0 / M_PI));

        custom_msgs::msg::DrivetrainControl drive_cmd;

        if (distance > distance_tolerance_) {
            // Proportional control for differential drive movement
            double left_speed = std::clamp(K_linear_ - heading_error * K_angular_, -60.0, 60.0);
            double right_speed = std::clamp(K_linear_ + heading_error * K_angular_, -60.0, 60.0);

            drive_cmd.lf_drive = left_speed;
            drive_cmd.lb_drive = left_speed;
            drive_cmd.rf_drive = right_speed;
            drive_cmd.rb_drive = right_speed;
        } else {
            // Reached the waypoint, adjust final orientation
            double final_heading_error = normalize_angle(wp.theta - theta_r_);

            if (std::abs(final_heading_error) > angle_tolerance_) {
                double turn_speed = std::clamp(final_heading_error * K_final_turn_, -40.0, 40.0);
                drive_cmd.lf_drive = -turn_speed;
                drive_cmd.lb_drive = -turn_speed;
                drive_cmd.rf_drive = turn_speed;
                drive_cmd.rb_drive = turn_speed;

                RCLCPP_INFO(this->get_logger(), "🔄 Adjusting final heading: %.2f rad (%.1f°)", final_heading_error, final_heading_error * (180.0 / M_PI));
            } else {
                // Final waypoint reached
                RCLCPP_INFO(this->get_logger(), "✅ Waypoint %ld reached!", current_waypoint_index_);
                current_waypoint_index_++;
                stop_robot();
                return;
            }
        }

        drive_pub_->publish(drive_cmd);
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void stop_robot() {
        custom_msgs::msg::DrivetrainControl stop_cmd;
        stop_cmd.lf_drive = 0;
        stop_cmd.lb_drive = 0;
        stop_cmd.rf_drive = 0;
        stop_cmd.rb_drive = 0;
        drive_pub_->publish(stop_cmd);
        RCLCPP_INFO(this->get_logger(), "All waypoints reached! Stopping.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
