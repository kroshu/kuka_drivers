cmake_minimum_required(VERSION 3.5)
project(kuka_sunrise)

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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kuka_driver_interfaces REQUIRED)
find_package(kroshu_ros2_core REQUIRED)
find_package(fri REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager_msgs)

include_directories(include)

add_library(fri_connection SHARED
  src/connection_helpers/fri_connection.cpp
  src/connection_helpers/tcp_connection.cpp)
target_link_libraries(fri_connection
  kroshu_ros2_core::communication_helpers)

add_library(kuka_fri_hw_interface SHARED
  src/hardware_interface.cpp
)
ament_target_dependencies(kuka_fri_hw_interface kuka_driver_interfaces rclcpp rclcpp_lifecycle hardware_interface)
target_link_libraries(kuka_fri_hw_interface fri::fri_client)

add_library(configuration_manager SHARED
  src/connection_helpers/configuration_manager.cpp)
ament_target_dependencies(configuration_manager kuka_driver_interfaces rclcpp rclcpp_lifecycle std_msgs std_srvs kroshu_ros2_core
  controller_manager_msgs)

add_executable(robot_manager_node
  src/robot_manager_node.cpp)
ament_target_dependencies(robot_manager_node kuka_driver_interfaces rclcpp rclcpp_lifecycle kroshu_ros2_core controller_manager_msgs)
target_link_libraries(robot_manager_node
  fri_connection
  configuration_manager)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface.xml)

install(TARGETS fri_connection kuka_fri_hw_interface robot_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(--language=c++)
  ament_pep257()
  ament_flake8()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()
  ament_xmllint()
endif()

ament_package()

# #####################
# # Project analysis ##
# #####################

# Additional options for project analysis
set(SONARQUBE_PACKAGES_FILE "none" CACHE STRING "Path to the file that contains the package source directories for SonarQube.")
option(TEST_COVERAGE "Generate test coverage reports upon testing." OFF)

if(NOT ${SONARQUBE_PACKAGES_FILE} MATCHES "none")
  file(APPEND ${SONARQUBE_PACKAGES_FILE} "${PROJECT_NAME};${PROJECT_SOURCE_DIR}\n")
  message(${SONARQUBE_PACKAGES_FILE})
endif()

if(TEST_COVERAGE)
  # Set build type to debug to avoid compiler optimization when checking coverage
  set(CMAKE_BUILD_TYPE Debug)

  # Include coverage report related functions
  list(APPEND CMAKE_MODULE_PATH "/usr/lib/cmake/CodeCoverage")
  include(CodeCoverage)

  # Append compiler flags to gcc for generating coverage notes
  append_coverage_compiler_flags()

  # Set coverage output dir to ${CMAKE_BINARY_DIR}/test_coverage
  set_coverage_output_dir()
endif()
