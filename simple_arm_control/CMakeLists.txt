cmake_minimum_required(VERSION 3.5.0)
project(simple_arm_control)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(simple_interface REQUIRED)
find_package(geometry_msgs REQUIRED)


install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)
include_directories(include)
add_compile_options(-g)

add_executable(moveit_controller src/moveit_controller.cpp src/simple_moveit.cpp include/service_handler.hpp include/shared.hpp)
ament_target_dependencies(moveit_controller
  rclcpp
  moveit_ros_planning_interface
  gazebo_msgs
  simple_interface
  geometry_msgs
)
add_executable(moveit_throw_controller src/moveit_throw_controller.cpp src/simple_moveit.cpp include/service_handler.hpp include/shared.hpp)
ament_target_dependencies(moveit_throw_controller
  rclcpp
  moveit_ros_planning_interface
  gazebo_msgs
  simple_interface
  geometry_msgs
)

add_executable(moveit_collision src/moveit_collision.cpp include/service_handler.hpp include/shared.hpp)
ament_target_dependencies(moveit_collision
  rclcpp
  moveit_ros_planning_interface
  gazebo_msgs
  simple_interface
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

## Install
install(TARGETS moveit_controller moveit_collision moveit_throw_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}/)
    
ament_package()
