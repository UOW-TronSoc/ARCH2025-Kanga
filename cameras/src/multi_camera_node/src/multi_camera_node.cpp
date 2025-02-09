#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

/*
 * A simple ROS2 node that captures frames from multiple cameras and publishes them.
 * - Each camera is published at 15 FPS (configurable).
 * - Resolutions are set to 480p (640x480).
 * - The node uses a single timer that fires at (number_of_cameras * 15) FPS.
 * - On each timer callback, only one camera is captured and published in a round-robin fashion.
 */

class MultiCameraNode : public rclcpp::Node
{
public:
  explicit MultiCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("multi_camera_node", options),
    current_camera_index_(0)
  {
    // Declare (and optionally get) private parameters for resolution and fps
    this->declare_parameter<int>("width", 640);
    this->declare_parameter<int>("height", 480);
    this->declare_parameter<int>("fps", 5);

    // Read the parameters
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    fps_ = this->get_parameter("fps").as_int();

    RCLCPP_INFO(this->get_logger(), "Using resolution %dx%d at %d FPS.", width_, height_, fps_);

    // Here you can specify the device indices or device paths.
    // For example, if you have 4 cameras: 2 RealSense enumerated as /dev/video0, /dev/video1
    // and 2 generic webcams enumerated as /dev/video2, /dev/video3, you could list them like so:
    camera_device_paths_ = {
      "/dev/video4",  // Realsense D435i #1 (RGB)
      // "/dev/video10",  // Realsense D435i #2 (RGB)
      // "/dev/video12",  // Generic Webcam #1
      // "/dev/video14",   // Generic Webcam #2
    };
// 
    // For each device, create a VideoCapture and a publisher
    for (size_t i = 0; i < camera_device_paths_.size(); i++) {
      cv::VideoCapture cap;
      if (!cap.open(camera_device_paths_[i], cv::CAP_V4L2)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera: %s", camera_device_paths_[i].c_str());
        // You may choose to throw or continue; here we'll continue with best-effort
      } else {
        // Set resolution and fps
        // if (i == 4) {  
        //   cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        //   cap.set(cv::CAP_PROP_BUFFERSIZE, 3);
        // }
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap.set(cv::CAP_PROP_FPS,         fps_);
        RCLCPP_INFO(this->get_logger(), "Opened camera %s successfully.", camera_device_paths_[i].c_str());
      }
      caps_.push_back(std::move(cap));

      // Create a unique publisher for this camera
      std::string topic_name = "/camera_" + std::to_string(i) + "/image_raw";
      auto pub = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
      publishers_.push_back(pub);
    }

    // If we want each camera to effectively publish at 'fps_' but in a staggered manner,
    // we run the timer at (number_of_cameras_ * fps_).
    auto total_rate = camera_device_paths_.size() * fps_;
    if (total_rate == 0) {
      RCLCPP_ERROR(this->get_logger(), "No cameras to capture, or invalid FPS. Exiting...");
      rclcpp::shutdown();
      return;
    }

    double timer_period = 1.0 / static_cast<double>(total_rate); // seconds
    RCLCPP_INFO(this->get_logger(), "Creating timer at %.2f Hz for round-robin capturing.",
                1.0 / timer_period);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(timer_period),
      std::bind(&MultiCameraNode::captureAndPublishCallback, this)
    );
  }

private:
  void captureAndPublishCallback()
  {
    if (caps_.empty() || publishers_.empty()) {
      return;
    }

    // Only capture from the current camera index
    size_t idx = current_camera_index_;
    if (idx >= caps_.size()) {
      // Safety check; should not happen if code is consistent
      RCLCPP_WARN(this->get_logger(), "Camera index out of range.");
      return;
    }

    auto & cap = caps_[idx];
    if (cap.isOpened()) {
      cv::Mat frame;
      cap >> frame; // read a new frame from camera
      if (!frame.empty()) {
        // Convert to ROS Image message
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_" + std::to_string(idx);

        // Use cv_bridge to convert OpenCV image (BGR) to sensor_msgs/Image
        cv_bridge::CvImage cv_image(header, "bgr8", frame);
        auto img_msg = cv_image.toImageMsg();

        // Publish
        publishers_[idx]->publish(*img_msg);
      } else {
        RCLCPP_WARN(this->get_logger(), "Empty frame from camera index %ld.", idx);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Camera at index %ld is not opened.", idx);
    }

    // Move to the next camera in a round-robin fashion
    current_camera_index_ = (current_camera_index_ + 1) % caps_.size();
  }

  // Parameters
  int width_, height_, fps_;

  // List of device paths (could be /dev/videoX or RealSense enumerations)
  std::vector<std::string> camera_device_paths_;

  // For capturing frames
  std::vector<cv::VideoCapture> caps_;

  // Publishers for each camera
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;

  // Timer for round-robin capture
  rclcpp::TimerBase::SharedPtr timer_;

  // Keeps track of which camera index we are capturing from in round-robin
  size_t current_camera_index_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
