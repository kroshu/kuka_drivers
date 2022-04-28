cmake_minimum_required(VERSION 3.5)
project(robot_control)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kuka_sunrise_interfaces REQUIRED)
find_package(kroshu_ros2_core REQUIRED)

include_directories(include ${kroshu_ros2_core_INCLUDE_DIRS})

add_library(joint_controller_base
  include/robot_control/joint_controller_base.hpp
  src/robot_control/joint_controller_base.cpp)
ament_target_dependencies(joint_controller_base rclcpp sensor_msgs kuka_sunrise_interfaces kroshu_ros2_core)
target_link_libraries(joint_controller_base ${kroshu_ros2_core_LIBRARIES})

add_executable(joint_controller
    src/robot_control/joint_controller.cpp)
ament_target_dependencies(joint_controller rclcpp sensor_msgs kuka_sunrise_interfaces)
target_link_libraries(joint_controller joint_controller_base)

install(TARGETS joint_controller
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
