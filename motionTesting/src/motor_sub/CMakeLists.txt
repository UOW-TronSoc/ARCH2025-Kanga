cmake_minimum_required(VERSION 3.5)
project(motor_sub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Enable warnings for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(odrive_can REQUIRED)

# Add executables
add_executable(motorVel src/motorVel.cpp)
ament_target_dependencies(motorVel rclcpp std_msgs odrive_can)

add_executable(motorState src/motorState.cpp)
ament_target_dependencies(motorState rclcpp std_msgs odrive_can)

add_executable(motorSubscriber src/motorSubscriber.cpp)
ament_target_dependencies(motorSubscriber rclcpp std_msgs odrive_can)

# Install executables
install(TARGETS
  motorVel
  motorState
  motorSubscriber
  DESTINATION lib/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(rclcpp std_msgs odrive_can)

ament_package()
