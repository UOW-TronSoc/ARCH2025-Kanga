cmake_minimum_required(VERSION 3.8)
project(auto_mode)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  # Ensure nav_msgs is found
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)  # Required for waypoints
find_package(custom_msgs REQUIRED)    # Ensure it's installed

# Executable: Auto Mode
add_executable(auto_mode src/auto_mode.cpp)
ament_target_dependencies(auto_mode rclcpp std_msgs custom_msgs nav_msgs)

# Executable: IMU Subscriber
add_executable(d435i_imu_subscriber src/d435i_imu_subscriber.cpp)
ament_target_dependencies(d435i_imu_subscriber rclcpp sensor_msgs)

# Executable: Odometry Subscriber
add_executable(odom_listener src/odom_listener.cpp)
ament_target_dependencies(odom_listener rclcpp nav_msgs tf2_ros)

# Executable: Waypoint Navigation
add_executable(waypoint_navigation src/waypoint_navigation.cpp)
ament_target_dependencies(waypoint_navigation rclcpp nav_msgs geometry_msgs tf2_ros custom_msgs)

# Install Targets
install(TARGETS
  auto_mode
  d435i_imu_subscriber
  odom_listener
  waypoint_navigation
  DESTINATION lib/${PROJECT_NAME}
)

# Testing (Linter)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
