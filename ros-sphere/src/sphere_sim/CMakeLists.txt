cmake_minimum_required(VERSION 3.8)
project(sphere_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(sphere_controller src/sphere_controller.cpp)
ament_target_dependencies(sphere_controller rclcpp geometry_msgs)

install(TARGETS sphere_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()