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

option(MOCK_FRI "Use mock classes instead of the FRI Client Library" ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kuka_sunrise_interfaces REQUIRED)
find_package(kroshu_ros2_core REQUIRED)

include_directories(include)

if(NOT MOCK_FRI)
  find_library(fri_client_library
    libFRIClient.a
    PATHS /home/rosdeveloper/Documents/fri_client_source/lib)
  message("Using real fri lib")
else()
  add_library(fri_client
    src/fri_client_mock/friClientApplication_mock.cpp
    src/fri_client_mock/friLBRClient_mock.cpp
    src/fri_client_mock/friLBRCommand_mock.cpp
    src/fri_client_mock/friLBRState_mock.cpp
    src/fri_client_mock/friTransformationClient_mock.cpp
    src/fri_client_mock/friUdpConnection_mock.cpp)
  set(fri_client_library fri_client)
  message("Using mock fri lib")
endif()


add_library(internal SHARED
  include/kuka_sunrise/internal/activatable_interface.hpp
  include/kuka_sunrise/internal/serialization.hpp
  include/kuka_sunrise/internal/service_tools.hpp)
ament_target_dependencies(internal rclcpp)
set_target_properties(internal PROPERTIES LINKER_LANGUAGE CXX)

ament_export_targets(internal HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)


add_library(robot_manager SHARED
  src/kuka_sunrise/robot_manager.cpp
  src/kuka_sunrise/tcp_connection.cpp)

add_library(robot_control_client SHARED
  src/kuka_sunrise/robot_control_client.cpp
  src/kuka_sunrise/robot_observer.cpp
  src/kuka_sunrise/robot_commander.cpp)
ament_target_dependencies(robot_control_client kuka_sunrise_interfaces rclcpp rclcpp_lifecycle std_msgs sensor_msgs std_srvs)

add_library(configuration_manager SHARED
  src/kuka_sunrise/configuration_manager.cpp)
ament_target_dependencies(configuration_manager kuka_sunrise_interfaces rclcpp rclcpp_lifecycle std_msgs std_srvs kroshu_ros2_core)


add_executable(robot_manager_node
  src/kuka_sunrise/robot_manager_node.cpp)
ament_target_dependencies(robot_manager_node kuka_sunrise_interfaces rclcpp rclcpp_lifecycle kroshu_ros2_core)
target_link_libraries(robot_manager_node
  robot_manager
  configuration_manager)

add_executable(robot_control_node
  src/kuka_sunrise/robot_control_node.cpp)
ament_target_dependencies(robot_control_node kuka_sunrise_interfaces rclcpp rclcpp_lifecycle std_msgs sensor_msgs std_srvs kroshu_ros2_core)
target_link_libraries(robot_control_node
  robot_control_client
  ${fri_client_library})

add_executable(position_controller_node
  test/position_controller.cpp)
ament_target_dependencies(position_controller_node rclcpp sensor_msgs)

add_executable(torque_controller_node
  test/torque_controller.cpp)
ament_target_dependencies(torque_controller_node rclcpp sensor_msgs)

add_executable(lifecycle_client_node
  test/lifecycle_client_test.cpp)
ament_target_dependencies(lifecycle_client_node rclcpp rclcpp_lifecycle std_msgs)


install(TARGETS robot_manager robot_control_client robot_manager_node robot_control_node position_controller_node torque_controller_node
        lifecycle_client_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/kuka_sunrise/internal DESTINATION include/kuka_sunrise)
install(
  TARGETS internal
  EXPORT internal
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)

  # _ament_cmake_python_register_environment_hook()


  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  # install custom copyright name
  install(CODE
    "execute_process(
      COMMAND
        \"${PYTHON_EXECUTABLE}\" \"${PROJECT_SOURCE_DIR}/setup.py\"
        \"egg_info\" \"--egg-base\" \"${PROJECT_BINARY_DIR}\"
        \"build\" \"--build-base\" \"${PROJECT_BINARY_DIR}/build\"
        \"install\" \"--prefix\" \"${CMAKE_INSTALL_PREFIX}\"
        \"--record\" \"${PROJECT_BINARY_DIR}/install_copyright.log\"
        \"--single-version-externally-managed\"
      WORKING_DIRECTORY \"${PROJECT_SOURCE_DIR}\")"
  )
  file(GLOB fri_headers
    LIST_DIRECTORIES FALSE
    RELATIVE "${PROJECT_SOURCE_DIR}/include/fri_client"
    include/fri_client/*)

  ament_copyright(--exclude ${fri_headers})
#    friClientApplication.h
#    friClientIf.h
#    friConnectionIf.h
#    friException.h
#    friLBRClient.h
#    friLBRCommand.h
#    friLBRState.h
#    friTransformationClient.h
#    friUdpConnection.h)
  ament_cppcheck(--language=c++)
  ament_pep257()
  ament_flake8()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify(--exclude ${fri_headers})
  ament_xmllint()
endif()

ament_package()

######################
## Project analysis ##
######################

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

if(TEST_COVERAGE)
  # Define a coverage target with some libs and executables to check
  # NAME    Name of coverage target. Default: coverage. Only works with default currently.
  # TARGETS Library and executable targets for which coverage reports are requested.
  # add_coverage_gcov(NAME coverage TARGETS mypkg_lib mypkg_node)
endif()
