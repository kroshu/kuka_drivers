cmake_minimum_required(VERSION 3.5)
project(kuka_iiqka_eac_driver)

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
find_package(kuka_drivers_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

if(NOT MOCK_KUKA_LIBS)
  find_package(os-core-udp-communication REQUIRED)
  find_package(motion-services-ecs-proto-api-cpp REQUIRED)
  find_package(motion-external-proto-api-nanopb REQUIRED)
  find_package(motion-services-ecs-proto-api-nanopb REQUIRED)
  find_package(nanopb-helpers REQUIRED)
  message("Using real kuka libs")
  add_definitions(-DNON_MOCK_SETUP)
else()
  find_package(Protobuf)
  include_directories(include/mock)
  add_library(os-core-udp-communication SHARED
    src/mock/os-core-udp-communication/udp_replier_mock.cpp
    src/mock/os-core-udp-communication/udp_socket_mock.cpp)
  add_library(kuka::os-core-udp-communication ALIAS os-core-udp-communication)
  install(TARGETS os-core-udp-communication
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )

  add_library(nanopb-helpers INTERFACE)
  target_include_directories(nanopb-helpers INTERFACE include/mock/nanopb-helpers)
  target_link_libraries(nanopb-helpers INTERFACE protobuf::libprotobuf)
  add_library(kuka::nanopb-helpers ALIAS nanopb-helpers)

  add_library(motion-services-ecs-proto-api-cpp INTERFACE)
  target_include_directories(motion-services-ecs-proto-api-cpp INTERFACE include/mock/kuka)

  add_library(motion-external-proto-api-nanopb INTERFACE)
  target_include_directories(motion-external-proto-api-nanopb INTERFACE include/mock/nanopb/kuka/motion/external)
  target_link_libraries(motion-external-proto-api-nanopb INTERFACE protobuf::libprotobuf)

  add_library(motion-services-ecs-proto-api-nanopb INTERFACE)
  target_include_directories(motion-services-ecs-proto-api-nanopb INTERFACE include/mock/nanopb/kuka/ecs/v1)
  target_link_libraries(motion-external-proto-api-nanopb INTERFACE motion-external-proto-api-nanopb)
  message("Using mock kuka libs")
  remove_definitions(-DNON_MOCK_SETUP)
endif()

add_library(${PROJECT_NAME} SHARED
  src/hardware_interface.cpp
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "KUKA_IIQKA_EAC_DRIVER_BUILDING_LIBRARY")

ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs hardware_interface kuka_drivers_core)
target_link_libraries(${PROJECT_NAME} motion-external-proto-api-nanopb motion-services-ecs-proto-api-cpp
  motion-services-ecs-proto-api-nanopb yaml-cpp kuka::os-core-udp-communication kuka::nanopb-helpers)


add_executable(robot_manager_node
  src/robot_manager_node.cpp)
ament_target_dependencies(robot_manager_node rclcpp kuka_drivers_core sensor_msgs controller_manager_msgs)
target_link_libraries(robot_manager_node kuka_drivers_core::communication_helpers motion-services-ecs-proto-api-cpp)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface.xml)

install(TARGETS ${PROJECT_NAME} robot_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/test_driver_startup.py)
endif()

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
