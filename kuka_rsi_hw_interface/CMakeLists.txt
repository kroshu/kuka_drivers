cmake_minimum_required(VERSION 3.5)
project(kuka_rsi_hw_interface)

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
find_package(kroshu_ros2_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(pluginlib REQUIRED)

find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)

include_directories(include ${TinyXML2_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/kuka_hardware_interface.cpp
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs hardware_interface)
target_link_libraries(${PROJECT_NAME} tinyxml)



add_executable(robot_manager_node
  src/robot_manager_node.cpp)
ament_target_dependencies(robot_manager_node rclcpp kroshu_ros2_core sensor_msgs controller_manager_msgs)
target_link_libraries(robot_manager_node kroshu_ros2_core::communication_helpers)

pluginlib_export_plugin_description_file(hardware_interface kuka_rsi_hw_interface.xml)

install(TARGETS ${PROJECT_NAME} robot_manager_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_cppcheck(--language=c++)
  ament_pep257()
  ament_flake8()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify(--language=C++)
  ament_xmllint(--exclude ros_rsi.rsi.xml)
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
