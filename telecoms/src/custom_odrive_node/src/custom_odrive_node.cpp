#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "std_msgs/msg/string.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <signal.h>
#include <mutex>

#include "custom_msgs/msg/drivetrain_feedback.hpp"
#include "custom_msgs/msg/drivetrain_control.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using namespace std;

// enum NodeIds : uint64_t {
//     LeftFrontWheel = 1,
//     LeftBackWheel = 2,
//     RightBackWheel = 3,
//     RightFrontWheel = 4,
// };

class ODriveCANNode : public rclcpp::Node {
public:
    ODriveCANNode() : Node("odrive_can_node"), can_socket_(-1) {

        subscription_ = this->create_subscription<custom_msgs::msg::DrivetrainControl>(
            "drive_commands", 10,
            std::bind(&ODriveCANNode::receive_message, this, std::placeholders::_1));

        publisher_ = this->create_publisher<custom_msgs::msg::DrivetrainFeedback>("drivetrain_feedback", 10);
        log_publisher_ = this->create_publisher<std_msgs::msg::String>("rover_logs", 10);
        // timer_ = this->create_wall_timer(10ms, std::bind(&ODriveCANNode::publish_message, this));

        if (!setup_can_interface("can1")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set up CAN interface");
            rclcpp::shutdown();
        } else {
            change_state(8, 1);
            change_state(8, 2);
            change_state(8, 3);
            change_state(8, 4);


            // velocity_callback(-20, 1);
            // velocity_callback(-20, 2);
            // velocity_callback(20, 3);
            // velocity_callback(20, 4);
        }
    }

    ~ODriveCANNode() {
        change_state(1, 1);
        change_state(1, 2);
        change_state(1, 3);
        change_state(1, 4);
        if (can_socket_ != -1) {
            close(can_socket_);
        }
    }

    void change_state(int state, int node_id) {
        RCLCPP_INFO(this->get_logger(), "Changing Node %d to state %d", node_id, state);
        auto log_message = std_msgs::msg::String();
        RCLCPP_INFO(this->get_logger(), "1");
        log_message.data = "Changing Node " + std::to_string(node_id) + " to state " + std::to_string(state);
        RCLCPP_INFO(this->get_logger(), "2");
        log_publisher_->publish(log_message);
        RCLCPP_INFO(this->get_logger(), "3");
        struct can_frame frame;
        frame.can_id = (node_id << 5) | 0x07;
        frame.can_dlc = 4;
        uint32_t state_val = state;
        std::memcpy(frame.data, &state_val, sizeof(state_val));
        write(can_socket_, &frame, sizeof(struct can_frame));
    }

    void velocity_callback(float velocity, int node_id) {
        velocity = std::max(-max_speed_, std::min(max_speed_, velocity));
        struct can_frame frame;
        frame.can_id = (node_id << 5) | 0x0d;
        frame.can_dlc = 8;

        {
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            std::memcpy(frame.data, &velocity, sizeof(float));
            float torque_feedforward = 0.0;
            std::memcpy(frame.data + 4, &torque_feedforward, sizeof(float));
        }

        write(can_socket_, &frame, sizeof(struct can_frame));
        // RCLCPP_INFO(this->get_logger(), "Sent velocity: %.3f turns/s to node %d", velocity, node_id);
    }

private:
    int can_socket_;
    std::mutex ctrl_msg_mutex_;
    rclcpp::Subscription<custom_msgs::msg::DrivetrainControl>::SharedPtr subscription_;
    rclcpp::Publisher<custom_msgs::msg::DrivetrainFeedback>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float max_speed_ = 10.0f;

    bool setup_can_interface(const std::string &interface) {
        struct ifreq ifr;
        struct sockaddr_can addr;

        if ((can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            perror("Error while opening CAN socket");
            return false;
        }

        std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            perror("Error getting interface index");
            return false;
        }

        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("Error binding socket");
            return false;
        }
        return true;
    }

    void receive_message(const custom_msgs::msg::DrivetrainControl::SharedPtr msg)
    {
        auto time = chrono::high_resolution_clock::now();
        int64_t duration_since_epoch = chrono::duration_cast<chrono::nanoseconds>(time.time_since_epoch()).count();

        int64_t latency = duration_since_epoch - msg->epoch_time;
        RCLCPP_INFO(this->get_logger(), "Left Front: '%d', Left Back: '%d', 'Right Back: '%d', Right Front: '%d'", msg->lf_drive, msg->lb_drive, msg->rb_drive, msg->rf_drive);

        auto log_message = std_msgs::msg::String();

        log_message.data = "Left Front: " + std::to_string(msg->lf_drive) + ", Left Back: " + std::to_string(msg->lb_drive) + ", 'Right Back: " + std::to_string(msg->rb_drive) + ", Right Front: "  + std::to_string(msg->rf_drive);

        log_publisher_->publish(log_message);

        if (abs(msg->lf_drive) > 10) {
            velocity_callback(msg->lf_drive * -0.01 * max_speed_, 1);
        } else {
            velocity_callback(0, 1);
        }

        if (abs(msg->lb_drive) > 10) {
            velocity_callback(msg->lb_drive * -0.01 * max_speed_, 2);
        } else {
            velocity_callback(0, 2);
        }

        if (abs(msg->rb_drive) > 10) {
            velocity_callback(msg->rb_drive * 0.01 * max_speed_, 3);
        } else {
            velocity_callback(0, 3);
        }

        if (abs(msg->rf_drive) > 10) {
            velocity_callback(msg->rf_drive * 0.01 * max_speed_, 4);
        } else {
            velocity_callback(0, 4);
        }

        // velocity_callback(msg->lf_drive * -0.01 * max_speed_, 1);
        // velocity_callback(msg->lb_drive * -0.01 * max_speed_, 2);
        // velocity_callback(msg->rb_drive * 0.01 * max_speed_, 3);
        // velocity_callback(msg->rf_drive * 0.01 * max_speed_, 4);

        // velocity_callback(-5, 1);
        // velocity_callback(-5, 2);
        // velocity_callback(5, 3);
        // velocity_callback(5, 4);
    }

    void publish_message()                                                                          
    {
        auto drivetrainFeedbackMessage = custom_msgs::msg::DrivetrainFeedback();
        struct can_frame frame;
        // float pos, vel;
        while (true) {
            if (read(can_socket_, &frame, sizeof(struct can_frame)) > 0) {
                if (frame.can_id == (1 << 5) | 0x09) { // Get_Encoder_Estimates
                    std::memcpy(&drivetrainFeedbackMessage.wheel_position[1], frame.data, sizeof(float));
                    std::memcpy(&drivetrainFeedbackMessage.wheel_velocity[1], frame.data + 4, sizeof(float));
                    // RCLCPP_INFO(this->get_logger(), "Node %d - pos: %.3f [turns], vel: %.3f [turns/s]", node_id, pos, vel);
                } else if (frame.can_id == (2 << 5) | 0x09) { // Get_Encoder_Estimates
                    std::memcpy(&drivetrainFeedbackMessage.wheel_position[2], frame.data, sizeof(float));
                    std::memcpy(&drivetrainFeedbackMessage.wheel_velocity[2], frame.data + 4, sizeof(float));
                    // RCLCPP_INFO(this->get_logger(), "Node %d - pos: %.3f [turns], vel: %.3f [turns/s]", node_id, pos, vel);
                } else if (frame.can_id == (3 << 5) | 0x09) { // Get_Encoder_Estimates
                    std::memcpy(&drivetrainFeedbackMessage.wheel_position[3], frame.data, sizeof(float));
                    std::memcpy(&drivetrainFeedbackMessage.wheel_velocity[3], frame.data + 4, sizeof(float));
                    // RCLCPP_INFO(this->get_logger(), "Node %d - pos: %.3f [turns], vel: %.3f [turns/s]", node_id, pos, vel);
                } else if (frame.can_id == (4 << 5) | 0x09) { // Get_Encoder_Estimates
                    std::memcpy(&drivetrainFeedbackMessage.wheel_position[4], frame.data, sizeof(float));
                    std::memcpy(&drivetrainFeedbackMessage.wheel_velocity[4], frame.data + 4, sizeof(float));
                    // RCLCPP_INFO(this->get_logger(), "Node %d - pos: %.3f [turns], vel: %.3f [turns/s]", node_id, pos, vel);
                }
            }
        }
        // auto message = custom_msgs::msg::DrivetrainFeedback();

        auto time = chrono::high_resolution_clock::now();
        int64_t duration_since_epoch = chrono::duration_cast<chrono::nanoseconds>(time.time_since_epoch()).count();

        drivetrainFeedbackMessage.epoch_time = duration_since_epoch;
        // message.wheel_position = {0.0, 1.0, 2.0, 3.0};
        // message.wheel_velocity = {4.0, 5.0, 6.0, 7.0};
        drivetrainFeedbackMessage.wheel_torque = {0.0, 0.0, 0.0, 0.0};

        RCLCPP_INFO(this->get_logger(), "Publishing DrivetrainFeedback - Epoch Time: %ld", drivetrainFeedbackMessage.epoch_time);

        auto log_message = std_msgs::msg::String();

        log_message.data = "Publishing DrivetrainFeedback - Epoch Time: " + std::to_string(drivetrainFeedbackMessage.epoch_time);

        log_publisher_->publish(log_message);
        publisher_->publish(drivetrainFeedbackMessage);
    }
    
};

void signal_handler(int signum) {
    rclcpp::shutdown();
}

int main(int argc, char **argv) {
    signal(SIGINT, signal_handler);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ODriveCANNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
