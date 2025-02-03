#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
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

class ODriveCANNode : public rclcpp::Node {
public:
    ODriveCANNode() : Node("odrive_can_node"), can_socket_(-1), node_id_(3) {
        velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "cmd_vel", 10, std::bind(&ODriveCANNode::velocity_callback, this, std::placeholders::_1)
        );
        
        if (!setup_can_interface("can1")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set up CAN interface.");
            rclcpp::shutdown();
        } else {
            change_state(8); // Enter closed loop control
        }
    }

    ~ODriveCANNode() {
        change_state(1); // Set state to 1 on shutdown
        if (can_socket_ != -1) {
            close(can_socket_);
        }
    }

private:
    int can_socket_;
    int node_id_;
    std::mutex ctrl_msg_mutex_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;

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

    void change_state(int state) {
        struct can_frame frame;
        frame.can_id = (node_id_ << 5) | 0x07;
        frame.can_dlc = 4;
        uint32_t state_val = state;
        std::memcpy(frame.data, &state_val, sizeof(state_val));
        write(can_socket_, &frame, sizeof(struct can_frame));
        
        // Wait for state confirmation
        struct can_frame response;
        while (true) {
            if (read(can_socket_, &response, sizeof(struct can_frame)) > 0) {
                if (response.can_id == ((node_id_ << 5) | 0x01)) { // Heartbeat message
                    uint32_t error;
                    uint8_t axis_state;
                    std::memcpy(&error, response.data, sizeof(uint32_t));
                    std::memcpy(&axis_state, response.data + 4, sizeof(uint8_t));
                    if (axis_state == state) {
                        RCLCPP_INFO(this->get_logger(), "Axis state set to %d", state);
                        break;
                    }
                }
            }
        }
    }

    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        double velocity = std::max(-20.0, std::min(20.0, msg->data));
        struct can_frame frame;
        frame.can_id = (node_id_ << 5) | 0x0d;
        frame.can_dlc = 8;

        {
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            std::memcpy(frame.data, &velocity, sizeof(float));
            float torque_feedforward = 0.0;
            std::memcpy(frame.data + 4, &torque_feedforward, sizeof(float));
        }

        write(can_socket_, &frame, sizeof(struct can_frame));
        RCLCPP_INFO(this->get_logger(), "Sent velocity: %.3f turns/s", velocity);
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
