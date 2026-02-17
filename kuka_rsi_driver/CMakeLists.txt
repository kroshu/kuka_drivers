cmake_minimum_required(VERSION 3.5)
project(kuka_rsi_driver)

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

find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(kuka-external-control-sdk CONFIG REQUIRED)
find_package(kuka_drivers_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Create the library
add_library(
  ${PROJECT_NAME} SHARED
  src/hardware_interface_rsi_base.cpp
  src/hardware_interface_eki_rsi.cpp
  src/hardware_interface_rsi_only.cpp
  src/hardware_interface_mxa_rsi.cpp
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "KUKA_RSI_DRIVER_BUILDING_LIBRARY")
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${PROJECT_NAME} hardware_interface kuka_drivers_core kuka-external-control-sdk)
target_link_libraries(${PROJECT_NAME} Kuka::kuka-external-control-sdk)

# Shared library for robot_manager_base
add_library(robot_manager_base STATIC src/robot_manager_base.cpp)
ament_target_dependencies(robot_manager_base std_msgs kuka_drivers_core controller_manager_msgs)
target_link_libraries(robot_manager_base kuka_drivers_core::communication_helpers)

add_executable(robot_manager_node_extended src/robot_manager_node_extended.cpp)
target_link_libraries(robot_manager_node_extended PRIVATE robot_manager_base)

add_executable(robot_manager_node_rsi_only src/robot_manager_node_rsi_only.cpp)
target_link_libraries(robot_manager_node_rsi_only PRIVATE robot_manager_base)

pluginlib_export_plugin_description_file(hardware_interface eki_rsi_hardware_interface.xml)
pluginlib_export_plugin_description_file(hardware_interface rsi_hardware_interface.xml)
pluginlib_export_plugin_description_file(hardware_interface mxa_rsi_hardware_interface.xml)


install(
  TARGETS ${PROJECT_NAME} robot_manager_base robot_manager_node_extended robot_manager_node_rsi_only
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/test_driver_startup_rsi_only.py)
  add_launch_test(test/test_driver_startup_eki_rsi.py)
  add_launch_test(test/test_driver_startup_mxa_rsi.py)
  add_launch_test(test/test_driver_activation.py)
  add_launch_test(test/test_multi_robot_startup.py)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

install(DIRECTORY config launch test
  DESTINATION share/${PROJECT_NAME})

ament_package()
