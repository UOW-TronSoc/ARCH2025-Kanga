#include "rclcpp/rclcpp.hpp"
#include "odrive_can/srv/axis_state.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Ensure the correct number of arguments are provided
  if (argc != 2) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: axis_state_client <axis_requested_state>");
    return 1;
  }

  // Create a node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("axis_state_client");

  // Create a client for the AxisState service
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client =
    node->create_client<odrive_can::srv::AxisState>("/odrive_axis0/request_axis_state");

  // Create a request
  auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
  request->axis_requested_state = static_cast<uint32_t>(std::atoi(argv[1]));

  // Wait for the service to become available
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
  }

  // Send the request asynchronously
  auto result = client->async_send_request(request);

  // Wait for the result
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = result.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call successful:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Active errors: %u", response->active_errors);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Axis state: %u", response->axis_state);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Procedure result: %u", response->procedure_result);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /odrive_axis0/request_axis_state");
  }

  rclcpp::shutdown();
  return 0;
}
