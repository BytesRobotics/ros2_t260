cmake_minimum_required(VERSION 3.5)
project(ros2_t260)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(map_msgs REQUIRED)

find_package(realsense2)
if(NOT realsense2_FOUND)
  message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing,
  please install it from
  https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()
find_library(realsense2_LIBRARIES realsense2)
find_path(realsense2_INCLUDE_DIRS librealsense2/rs.hpp)

add_executable(t260_node
  src/t260_node.cpp
  src/t260.cpp
)

include_directories(t260_node
  include
  ${realsense2_INCLUDE_DIRS}
)

ament_target_dependencies(t260_node
  rclcpp
  realsense2
  OpenCV
  cv_bridge
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  rclcpp_lifecycle
  lifecycle_msgs
  nav_msgs
  geometry_msgs
  map_msgs
)


install(TARGETS t260_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
