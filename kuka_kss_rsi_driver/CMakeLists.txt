cmake_minimum_required(VERSION 3.5)
project(kuka_kss_rsi_driver)

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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kuka_drivers_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(pluginlib REQUIRED)

find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)

include_directories(include ${TinyXML2_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/hardware_interface.cpp
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "KUKA_KSS_RSI_DRIVER_BUILDING_LIBRARY")
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs hardware_interface)
target_link_libraries(${PROJECT_NAME} tinyxml)



add_executable(robot_manager_node
  src/robot_manager_node.cpp)
ament_target_dependencies(robot_manager_node rclcpp kuka_drivers_core sensor_msgs controller_manager_msgs)
target_link_libraries(robot_manager_node kuka_drivers_core::communication_helpers)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface.xml)

install(TARGETS ${PROJECT_NAME} robot_manager_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/test_driver_startup.py)
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

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
