cmake_minimum_required(VERSION 3.5)
project(science_control)

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


add_executable(science_control src/science_control.cpp)
ament_target_dependencies(science_control rclcpp std_msgs custom_msgs)

add_executable(science_feedback src/science_feedback.cpp)
ament_target_dependencies(science_feedback rclcpp std_msgs custom_msgs)

install(TARGETS
  science_control
  science_feedback
  DESTINATION lib/${PROJECT_NAME})

  ament_package()
