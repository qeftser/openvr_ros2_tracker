cmake_minimum_required(VERSION 3.8)
project(openvr_tracker_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -g)
endif()

set(OpenVR_INCLUDE_DIR "$ENV{OPENVR_DIRECTORY}/headers")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenVR REQUIRED)


# ros stuff

add_executable(openvr_tracker_node src/openvr_tracker_node.cpp)
target_link_libraries(openvr_tracker_node OpenVR::OpenVR)
ament_target_dependencies(openvr_tracker_node rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs tf2)

install(
   TARGETS openvr_tracker_node
   DESTINATION lib/${PROJECT_NAME})
install(
   DIRECTORY launch config
   DESTINATION share/${PROJECT_NAME})



ament_package()
