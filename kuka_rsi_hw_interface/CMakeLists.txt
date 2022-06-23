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
find_package(pluginlib REQUIRED)

find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)

include_directories(include ${TinyXML2_INCLUDE_DIRS})

add_library(kuka_hardware_interface
  src/kuka_hardware_interface.cpp
)
ament_target_dependencies(kuka_hardware_interface rclcpp sensor_msgs hardware_interface)
target_link_libraries(kuka_hardware_interface tinyxml)

add_executable(robot_control_node
  src/robot_control_node.cpp)
ament_target_dependencies(robot_control_node rclcpp kroshu_ros2_core sensor_msgs)
target_link_libraries(robot_control_node kuka_hardware_interface)


install(TARGETS kuka_hardware_interface robot_control_node
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

ament_package()
