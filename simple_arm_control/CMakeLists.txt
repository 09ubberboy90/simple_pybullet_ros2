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
find_package(backward_ros REQUIRED)
find_package(simple_interface REQUIRED)


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
)
# add_executable(throw_moveit src/throw_moveit.cpp include/moveit.hpp)
# ament_target_dependencies(throw_moveit
#   rclcpp
#   rclcpp_action
#   control_msgs
#   moveit_ros_planning_interface
#   sim_action_server
# )

add_executable(moveit_collision src/moveit_collision.cpp include/service_handler.hpp include/shared.hpp)
ament_target_dependencies(moveit_collision
  rclcpp
  moveit_ros_planning_interface
  gazebo_msgs
  simple_interface
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

## Install
install(TARGETS moveit_controller  moveit_collision
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}/)
  
# install(
#   TARGETS simple_arm_control
#   EXPORT simple_arm_control     # <-- documentation prepends export_
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

# ament_export_include_directories(include)
# ament_export_libraries(simple_arm_control)
  
ament_package()
