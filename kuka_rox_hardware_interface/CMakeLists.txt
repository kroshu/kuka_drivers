cmake_minimum_required(VERSION 3.5)
project(kuka_rox_hw_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)

find_package(motion-external-proto-api-nanopb REQUIRED)
find_package(motion-services-ecs-proto-api-cpp REQUIRED)
find_package(motion-services-ecs-proto-api-nanopb REQUIRED)
find_package(os-core-udp-communication REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include /usr/include/kuka/os-core-udp-communication-0.0)  # TODO

add_library(kuka_rox_hw_interface SHARED
  src/rox_hardware_interface.cpp
)
ament_target_dependencies(kuka_rox_hw_interface rclcpp sensor_msgs hardware_interface)
target_link_libraries(kuka_rox_hw_interface motion-external-proto-api-nanopb motion-services-ecs-proto-api-cpp motion-services-ecs-proto-api-nanopb os-core-udp-communication yaml-cpp)

add_executable(rox_control_node
  src/rox_control_node.cpp)
ament_target_dependencies(rox_control_node rclcpp kroshu_ros2_core sensor_msgs controller_manager controller_manager_msgs)
target_link_libraries(rox_control_node kuka_rox_hw_interface)

pluginlib_export_plugin_description_file(hardware_interface kuka_rox_hw_interface.xml)

install(TARGETS kuka_rox_hw_interface rox_control_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY config launch
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

  ament_cppcheck(--language=c++)
  ament_pep257()
  ament_flake8()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify(--language=C++)
  ament_xmllint()
endif()

ament_package()
