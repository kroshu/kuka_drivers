cmake_minimum_required(VERSION 3.5)
project(teleop_guided_robot)

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
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(kuka_sunrise REQUIRED)
find_package(kroshu_ros2_core REQUIRED)

include_directories(include)

add_executable(keyboard_control
  src/control_logic/keyboard_control.cpp)
ament_target_dependencies(keyboard_control rclcpp rclcpp_lifecycle sensor_msgs geometry_msgs kroshu_ros2_core)

add_executable(system_manager
  src/control_system/system_manager.cpp)
ament_target_dependencies(system_manager rclcpp rclcpp_lifecycle std_msgs std_srvs)
target_link_libraries(system_manager kuka_sunrise::internal)

install(TARGETS keyboard_control system_manager
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
