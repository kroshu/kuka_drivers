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

# A few (transitive) dependencies are available only in one of the dependencies
# Set flag to suppress linker error
# TODO: find better solution
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--copy-dt-needed-entries")

set(MOCK_KUKA_LIBS TRUE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kroshu_ros2_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

if(NOT MOCK_KUKA_LIBS)
  find_package(os-core-udp-communication REQUIRED)
  find_package(motion-services-ecs-proto-api-cpp REQUIRED)
  find_package(motion-external-proto-api-nanopb REQUIRED)
  find_package(motion-services-ecs-proto-api-nanopb REQUIRED)
  find_package(nanopb-helpers REQUIRED)
  include_directories(/usr/include/kuka/os-core-udp-communication-0.0)
  include_directories(/usr/include/kuka/nanopb-helpers-0.1)
  message("Using real kuka libs")
  add_definitions(-DNON_MOCK_SETUP)
else()
  find_package(Protobuf)
  include_directories(include/mock)
  add_library(os-core-udp-communication SHARED
    src/mock/os-core-udp-communication/udp_replier_mock.cpp
    src/mock/os-core-udp-communication/udp_socket_mock.cpp)

  add_library(motion-services-ecs-proto-api-cpp INTERFACE)
  target_include_directories(motion-services-ecs-proto-api-cpp INTERFACE include/mock)
  target_link_libraries(motion-services-ecs-proto-api-cpp INTERFACE protobuf::libprotobuf)

  add_library(motion-external-proto-api-nanopb INTERFACE)
  target_include_directories(motion-external-proto-api-nanopb INTERFACE include/mock)

  add_library(motion-services-ecs-proto-api-nanopb INTERFACE)
  target_include_directories(motion-services-ecs-proto-api-nanopb INTERFACE include/mock)
  message("Using mock kuka libs")
  remove_definitions(-DNON_MOCK_SETUP)
endif()

add_library(kuka_rox_hw_interface SHARED
  src/rox_hardware_interface.cpp
)
ament_target_dependencies(kuka_rox_hw_interface rclcpp sensor_msgs hardware_interface)
target_link_libraries(kuka_rox_hw_interface motion-external-proto-api-nanopb motion-services-ecs-proto-api-cpp
  motion-services-ecs-proto-api-nanopb yaml-cpp os-core-udp-communication)

add_executable(rox_control_node
  src/rox_control_node.cpp)
ament_target_dependencies(rox_control_node rclcpp kroshu_ros2_core sensor_msgs controller_manager controller_manager_msgs)
target_link_libraries(rox_control_node kuka_rox_hw_interface)

add_executable(robot_manager_node
  src/robot_manager_node.cpp)
ament_target_dependencies(robot_manager_node rclcpp kroshu_ros2_core sensor_msgs controller_manager controller_manager_msgs)
target_link_libraries(robot_manager_node kroshu_ros2_core::communication_helpers motion-services-ecs-proto-api-cpp)

pluginlib_export_plugin_description_file(hardware_interface kuka_rox_hw_interface.xml)

install(TARGETS kuka_rox_hw_interface rox_control_node robot_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  file(GLOB_RECURSE mock_headers
    LIST_DIRECTORIES FALSE
    RELATIVE "${PROJECT_SOURCE_DIR}"
    include/mock/*)

  file(GLOB_RECURSE mock_src
    LIST_DIRECTORIES FALSE
    RELATIVE "${PROJECT_SOURCE_DIR}"
    src/mock/*)

  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright(--exclude ${mock_headers} ${mock_src})
  ament_cppcheck(--language=c++)
  ament_pep257()
  ament_flake8()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify(--exclude ${mock_headers} ${mock_src})
  ament_xmllint()
endif()

ament_package()
