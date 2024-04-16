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

list(APPEND CMAKE_PREFIX_PATH "~/.local/lib/cmake")

find_package(ament_cmake REQUIRED)
find_package(kuka_drivers_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(kuka-external-control-sdk CONFIG REQUIRED)
include_directories(include)


add_library(${PROJECT_NAME} SHARED
  src/hardware_interface.cpp
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "KUKA_IIQKA_EAC_DRIVER_BUILDING_LIBRARY")

ament_target_dependencies(${PROJECT_NAME} hardware_interface kuka_drivers_core kuka-external-control-sdk)
target_link_libraries(${PROJECT_NAME} Kuka::kuka-external-control-sdk)


add_executable(robot_manager_node
  src/robot_manager_node.cpp)
ament_target_dependencies(robot_manager_node std_msgs kuka_drivers_core controller_manager_msgs)
target_link_libraries(robot_manager_node kuka_drivers_core::communication_helpers Kuka::kuka-external-control-sdk)

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
ament_package()
