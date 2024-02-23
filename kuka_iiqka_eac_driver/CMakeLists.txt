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


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kuka_drivers_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager_msgs REQUIRED)

include_directories(include)

find_library(EXTERNAL_CONTROL_SDK kuka-external-control-sdk PATHS "~/.local/lib" REQUIRED)
find_library(EXTERNAL_CONTROL_SDK_PROTOBUF kuka-external-control-sdk-protobuf PATHS "~/.local/lib" REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
      "~/.local/include"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "KUKA_IIQKA_EAC_DRIVER_BUILDING_LIBRARY")

ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs hardware_interface kuka_drivers_core)
target_link_libraries(${PROJECT_NAME} ${EXTERNAL_CONTROL_SDK})


add_executable(robot_manager_node
  src/robot_manager_node.cpp)
ament_target_dependencies(robot_manager_node rclcpp kuka_drivers_core sensor_msgs controller_manager_msgs)
target_link_libraries(robot_manager_node kuka_drivers_core::communication_helpers ${EXTERNAL_CONTROL_SDK})

pluginlib_export_plugin_description_file(hardware_interface hardware_interface.xml)

install(TARGETS ${PROJECT_NAME} robot_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch test
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/test_driver_startup.py)
  add_launch_test(test/test_driver_activation.py)
  add_launch_test(test/test_multi_robot_startup.py)
endif()

ament_export_libraries(
  ${PROJECT_NAME}
)

# Hack to extend $LD_LIBRARY_PATH with ~/.local/include when sourcing
file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_environment_hooks/library_path.dsv
  "prepend-non-duplicate;LD_LIBRARY_PATH;~/.local/include")

ament_package()
