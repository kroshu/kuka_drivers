cmake_minimum_required(VERSION 3.5)
project(kuka_drivers_core)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(kuka-external-control-sdk CONFIG REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(kuka_drivers_core SHARED
  src/ros2_base_node.cpp
  src/ros2_base_lc_node.cpp
  src/parameter_handler.cpp
  src/controller_handler.cpp
)
ament_target_dependencies(kuka_drivers_core rclcpp rclcpp_lifecycle lifecycle_msgs)

add_executable(control_node
  src/control_node.cpp)
ament_target_dependencies(control_node rclcpp rclcpp_lifecycle controller_manager)

ament_export_targets(export_kuka_drivers_core HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle lifecycle_msgs)
ament_export_libraries(${PROJECT_NAME})

add_library(communication_helpers SHARED
  include/communication_helpers/ros2_control_tools.hpp
  include/communication_helpers/service_tools.hpp)
ament_target_dependencies(communication_helpers rclcpp)
set_target_properties(communication_helpers PROPERTIES LINKER_LANGUAGE CXX)

ament_export_targets(communication_helpers HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)


add_library(kuka_iiqka_drivers_core SHARED
  include/kuka_iiqka_drivers_core/event_observer.hpp
  include/kuka_iiqka_drivers_core/robot_manager_node_base.hpp
  src/kuka_iiqka_drivers_core/hardware_interface_base.cpp
  src/kuka_iiqka_drivers_core/robot_manager_node_base.cpp

)
ament_target_dependencies(kuka_iiqka_drivers_core rclcpp rclcpp_lifecycle lifecycle_msgs kuka-external-control-sdk controller_manager_msgs std_msgs hardware_interface)
set_target_properties(kuka_iiqka_drivers_core PROPERTIES LINKER_LANGUAGE CXX)
ament_export_targets(kuka_iiqka_drivers_core HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp controller_manager_msgs kuka-external-control-sdk)

target_link_libraries(kuka_iiqka_drivers_core Kuka::kuka-external-control-sdk communication_helpers kuka_drivers_core)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(TARGETS kuka_drivers_core
  EXPORT export_kuka_drivers_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(DIRECTORY include/communication_helpers DESTINATION include)
install(
  TARGETS communication_helpers
  EXPORT communication_helpers
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/kuka_iiqka_drivers_core DESTINATION include)
install(
  TARGETS kuka_iiqka_drivers_core
  EXPORT kuka_iiqka_drivers_core
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
install(TARGETS ${PROJECT_NAME} control_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

if(BUILD_TESTING)

endif()

ament_package()

set(SONARQUBE_PACKAGES_FILE "none" CACHE STRING "Path to the file that contains the package source directories for SonarQube.")
option(TEST_COVERAGE "Generate test coverage reports upon testing." OFF)

if(NOT ${SONARQUBE_PACKAGES_FILE} MATCHES "none")
  file(APPEND ${SONARQUBE_PACKAGES_FILE} "${PROJECT_NAME};${PROJECT_SOURCE_DIR}\n")
  message(${SONARQUBE_PACKAGES_FILE})
endif()

if(TEST_COVERAGE)
  set(CMAKE_BUILD_TYPE Debug)
  list(APPEND CMAKE_MODULE_PATH "/usr/lib/cmake/CodeCoverage")
  include(CodeCoverage)
  append_coverage_compiler_flags()
  set_coverage_output_dir()
endif()

if(TEST_COVERAGE)
endif()
