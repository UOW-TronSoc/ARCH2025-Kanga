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

class AutoDriveMode : public rclcpp::Node {
public:
    AutoDriveMode() : Node("autonomous_mode"), can_socket_(-1) {

        // subscription_ = this->create_subscription<custom_msgs::msg::DrivetrainControl>(
        //     "drive_commands", 10,
        //     std::bind(&AutoDriveMode::receive_message, this, std::placeholders::_1));

        publisher_ = this->create_publisher<custom_msgs::msg::DrivetrainFeedback>("drivetrain_feedback", 10);
        log_publisher_ = this->create_publisher<std_msgs::msg::String>("rover_logs", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&AutoDriveMode::processAutonomous, this));

        if (!setup_can_interface("can1")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set up CAN interface");
            rclcpp::shutdown();
        } else {
            change_state(8, 1);
            change_state(8, 2);
            change_state(8, 3);
            change_state(8, 4);
        }

        RCLCPP_INFO(this->get_logger(), "Starting Autonomous Navigation");

        auto log_message = std_msgs::msg::String();

        log_message.data = "Starting Autonomous Navigation";

        log_publisher_->publish(log_message);
    }

    ~AutoDriveMode() {

        RCLCPP_INFO(this->get_logger(), "Stopping Autonomous Navigation");

        auto log_message = std_msgs::msg::String();

        log_message.data = "Stopping Autonomous Navigation";

        log_publisher_->publish(log_message);

        change_state2(1, 1);
        change_state2(1, 2);
        change_state2(1, 3);
        change_state2(1, 4);
        if (can_socket_ != -1) {
            close(can_socket_);
        }
    }

    void change_state(int state, int node_id) {
        RCLCPP_INFO(this->get_logger(), "Changing Wheel CANBus Node %d to state %d. Motor Speed Cap: 10 t/s, Motor Acceleration Cap: 15 t/s/s.", node_id, state);
        auto log_message = std_msgs::msg::String();
        log_message.data = "Changing Wheel CANBus Node " + std::to_string(node_id) + " to state " + std::to_string(state) + ". Motor in in closed loop control. Motor Speed Cap: 10 t/s, Motor Acceleration Cap: 15 t/s/s.";
        log_publisher_->publish(log_message);
        struct can_frame frame;
        frame.can_id = (node_id << 5) | 0x07;
        frame.can_dlc = 4;
        uint32_t state_val = state;
        std::memcpy(frame.data, &state_val, sizeof(state_val));
        write(can_socket_, &frame, sizeof(struct can_frame));
    }

    void change_state2(int state, int node_id) {
        RCLCPP_INFO(this->get_logger(), "Changing Wheel CANBus Node %d to state %d. Motor in Idle Mode.", node_id, state);
        auto log_message = std_msgs::msg::String();
        log_message.data = "Changing Wheel CANBus Node " + std::to_string(node_id) + " to state " + std::to_string(state) + ". Motor in Idle Mode.";
        log_publisher_->publish(log_message);
        struct can_frame frame;
        frame.can_id = (node_id << 5) | 0x07;
        frame.can_dlc = 4;
        uint32_t state_val = state;
        std::memcpy(frame.data, &state_val, sizeof(state_val));
        write(can_socket_, &frame, sizeof(struct can_frame));
    }

    void velocity_callback(float velocity, int node_id) {
        velocity = std::max(-max_speed_, std::min(max_speed_, velocity));

        if (node_id < 3) {
            velocity*=-1;
        }
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
    // rclcpp::Subscription<custom_msgs::msg::DrivetrainControl>::SharedPtr subscription_;
    rclcpp::Publisher<custom_msgs::msg::DrivetrainFeedback>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int time = 0;
    float max_speed_ = 20.0f;

    int timeAcc = 200;

    int timeA = 7400; // start turn (anti)
    int timeB = timeAcc; // stop turn
    int timeC = 28400; // start forward
    int timeD = timeAcc; // stop forwars
    int timeE = 3100; // start turn (anti)
    int timeF = timeAcc; // stop turn
    int timeG = 27300; // start forward
    int timeH = timeAcc; // stop forward
    int timeI = 10500;  // start turn (clock)
    int timeJ = timeAcc; // stopm turn 
    int timeK = 16350; // start forward)
    int timeL = timeAcc; // stop forward

    int t1 = timeA;
    int t2 = t1 + timeB;
    int t3 = t2 + timeC;
    int t4 = t3 + timeD;
    int t5 = t4 + timeE;
    int t6 = t5 + timeF;
    int t7 = t6 + timeG;
    int t8 = t7 + timeH;
    int t9 = t8 + timeI;
    int t10 = t9 + timeJ;
    int t11 = t10 + timeK;
    int t12 = t11 + timeL;
    int t13 = t12 + 2000;


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

    void processAutonomous() {
        time += 10;

        auto log_message = std_msgs::msg::String();      

        if (time < t1) { // start turn (ant)
            velocity_callback(-10, 1);
            velocity_callback(-10, 2);
            velocity_callback(10, 3);
            velocity_callback(10, 4);
        } else if (time >= t1 && time < t2) { //stop turn
            velocity_callback(0, 1);
            velocity_callback(0, 2);
            velocity_callback(0, 3);
            velocity_callback(0, 4);
        } else if (time >= t2 && time < t3) { // start straight
            velocity_callback(10, 1);
            velocity_callback(10, 2);
            velocity_callback(10, 3);
            velocity_callback(10, 4);
        } else if (time >= t3 && time < t4) { // stop straight
            velocity_callback(0, 1);
            velocity_callback(0, 2);
            velocity_callback(0, 3);
            velocity_callback(0, 4);
        } else if (time >= t4 && time < t5) { // start turn (anti)
            velocity_callback(-10, 1);
            velocity_callback(-10, 2);
            velocity_callback(10, 3);
            velocity_callback(10, 4);
        } else if (time >= t5 && time < t6) { // stop straight
            velocity_callback(0, 1);
            velocity_callback(0, 2);
            velocity_callback(0, 3);
            velocity_callback(0, 4);
        } else if (time >= t6 && time < t7) { // start straight
            velocity_callback(10, 1);
            velocity_callback(10, 2);
            velocity_callback(10, 3);
            velocity_callback(10, 4);
        } else if (time >= t7 && time < t8) { // stop straight
            velocity_callback(0, 1);
            velocity_callback(0, 2);
            velocity_callback(0, 3);
            velocity_callback(0, 4);
        } else if (time >= t8 && time < t9) { //start turn (clock)
            velocity_callback(10, 1);
            velocity_callback(10, 2);
            velocity_callback(-10, 3);
            velocity_callback(-10, 4);
        } else if (time >= t9 && time < t10) { // stop turn 
            velocity_callback(0, 1);
            velocity_callback(0, 2);
            velocity_callback(0, 3);
            velocity_callback(0, 4);
        } else if (time >= t10 && time < t11) { //start straight
            velocity_callback(10, 1);
            velocity_callback(10, 2);
            velocity_callback(10, 3);
            velocity_callback(10, 4);
        } else if (time >= t11 && time < t12) { // stop straight
            velocity_callback(0, 1);
            velocity_callback(0, 2);
            velocity_callback(0, 3);
            velocity_callback(0, 4);
        }


        if (time == t1) {
            // Starting Turn (anti)
            RCLCPP_INFO(this->get_logger(), "Starting Anticlockwise Turn");
            log_message.data = "Starting Anticlockwise Turn";
        } else if (time == t2) {
            // Stopping Turn (anti)
            RCLCPP_INFO(this->get_logger(), "Stopping Anticlockwise Turn");
            log_message.data = "Stopping Anticlockwise Turn";
        } else if (time == t3) {
            // Starting Straight Drive
            RCLCPP_INFO(this->get_logger(), "Starting Straight Drive");
            log_message.data = "Starting Straight Drive";
        } else if (time == t4) {
            // Stopping Straight Drive
            RCLCPP_INFO(this->get_logger(), "Stopping Straight Drive");
            log_message.data = "Stopping Straight Drive";
        } else if (time == t5) {
            // Starting Turn (anti)
            RCLCPP_INFO(this->get_logger(), "Starting Anticlockwise Turn");
            log_message.data = "Starting Anticlockwise Turn";
        } else if (time == t6) {
            // Stopping Turn (anti)
            RCLCPP_INFO(this->get_logger(), "Stopping Anticlockwise Turn");
            log_message.data = "Stopping Anticlockwise Turn";
        } else if (time == t7) {
            // Starting Straight Drive
            RCLCPP_INFO(this->get_logger(), "Starting Straight Drive");
            log_message.data = "Starting Straight Drive";
        } else if (time == t8) {
            // Stopping Straight Drive
            RCLCPP_INFO(this->get_logger(), "Stopping Straight Drive");
            log_message.data = "Stopping Straight Drive";
        } else if (time == t9) {
            // Starting Turn (clockwise)
            RCLCPP_INFO(this->get_logger(), "Starting Clockwise Turn");
            log_message.data = "Starting Clockwise Turn";
        } else if (time == t10) {
            // Stopping Turn (clockwise)
            RCLCPP_INFO(this->get_logger(), "Stopping Clockwise Turn");
            log_message.data = "Stopping Clockwise Turn";
        } else if (time == t11) {
            // Starting Straight Drive
            RCLCPP_INFO(this->get_logger(), "Starting Straight Drive");
            log_message.data = "Starting Straight Drive";
        } else if (time == t12) {
            // Stopping Straight Drive
            RCLCPP_INFO(this->get_logger(), "Stopping Straight Drive");
            log_message.data = "Stopping Straight Drive";
        } else if (time == t13) {
            // Stopping Straight Drive
            rclcpp::shutdown();
        }

        if (!log_message.data.empty()) {
            log_publisher_->publish(log_message);
        }
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
    auto node = std::make_shared<AutoDriveMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
