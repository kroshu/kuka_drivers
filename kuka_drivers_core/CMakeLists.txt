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
  include/communication_helpers/serialization.hpp
  include/communication_helpers/service_tools.hpp)
ament_target_dependencies(communication_helpers rclcpp)
set_target_properties(communication_helpers PROPERTIES LINKER_LANGUAGE CXX)

ament_export_targets(communication_helpers HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

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

install(TARGETS ${PROJECT_NAME} control_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

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
