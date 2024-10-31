cmake_minimum_required(VERSION 3.5)
project(kuka_sunrise_fri_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# C++17 needed for string_view
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(kuka_driver_interfaces REQUIRED)
find_package(kuka_drivers_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager_msgs)
find_package(std_msgs)
find_package(std_srvs)

include_directories(include src/fri_client_sdk)

add_library(fri_connection SHARED
  src/connection_helpers/fri_connection.cpp
  src/connection_helpers/tcp_connection.cpp)
target_link_libraries(fri_connection
  kuka_drivers_core::communication_helpers)


file(GLOB fri_client_sources
  LIST_DIRECTORIES FALSE
  RELATIVE "${PROJECT_SOURCE_DIR}"
    src/fri_client_sdk/HWIFClientApplication.cpp
    src/fri_client_sdk/friClientApplication.cpp
    src/fri_client_sdk/friCommandMessageEncoder.cpp
    src/fri_client_sdk/friLBRClient.cpp
    src/fri_client_sdk/friLBRCommand.cpp
    src/fri_client_sdk/friLBRState.cpp
    src/fri_client_sdk/FRIMessages.pb.c
    src/fri_client_sdk/friMonitoringMessageDecoder.cpp
    src/fri_client_sdk/friTransformationClient.cpp
    src/fri_client_sdk/friUdpConnection.cpp
    src/fri_client_sdk/pb_frimessages_callbacks.c
)

# Add the Fast Robot Interface library
add_library(fri_client_sdk SHARED ${fri_client_sources})

file(GLOB private_headers
  LIST_DIRECTORIES FALSE
  RELATIVE "${PROJECT_SOURCE_DIR}"
  src/fri_client_sdk/friClientData.h
  src/fri_client_sdk/friCommandMessageEncoder.h
  src/fri_client_sdk/FRIMessages.pb.h
  src/fri_client_sdk/friMonitoringMessageDecoder.h
  src/fri_client_sdk/pb_frimessages_callbacks.h
)

target_link_libraries(fri_client_sdk PRIVATE protobuf-nanopb)

install(DIRECTORY include/fri_client_sdk DESTINATION include)
install(FILES ${private_headers} DESTINATION include)

add_library(${PROJECT_NAME} SHARED
  src/hardware_interface.cpp
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "KUKA_SUNRISE_FRI_DRIVER_BUILDING_LIBRARY")

ament_target_dependencies(${PROJECT_NAME} kuka_driver_interfaces hardware_interface kuka_drivers_core)
target_link_libraries(${PROJECT_NAME} fri_client_sdk fri_connection)

add_executable(robot_manager_node
  src/robot_manager_node.cpp)
ament_target_dependencies(robot_manager_node kuka_driver_interfaces kuka_drivers_core std_msgs std_srvs
  controller_manager_msgs)


pluginlib_export_plugin_description_file(hardware_interface hardware_interface.xml)

install(TARGETS ${PROJECT_NAME} robot_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS fri_connection fri_client_sdk
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY launch config test
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/test_driver_activation.py)
  add_launch_test(test/test_driver_startup.py)
  add_launch_test(test/test_multi_robot_startup.py)
endif()

ament_package()
