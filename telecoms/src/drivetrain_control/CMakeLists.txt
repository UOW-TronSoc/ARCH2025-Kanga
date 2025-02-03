cmake_minimum_required(VERSION 3.5)
project(drivetrain_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(motor_control src/motor_control.cpp)
ament_target_dependencies(motor_control rclcpp custom_msgs std_msgs)

# add_executable(motor_feedback src/motor_feedback.cpp)
# ament_target_dependencies(motor_feedback rclcpp custom_msgs std_msgs)

install(TARGETS
  motor_control
  # motor_feedback
  DESTINATION lib/${PROJECT_NAME})

  ament_package()
