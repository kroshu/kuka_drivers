cmake_minimum_required(VERSION 3.8)
project(kuka_nrt_message_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kuka_drivers_core REQUIRED)
find_package(kuka_driver_interfaces REQUIRED)

add_library(kuka_nrt_message_handler SHARED src/kuka_nrt_message_handler.cpp)
add_library(kuka_nrt_message_handler::kuka_nrt_message_handler ALIAS kuka_nrt_message_handler)
target_compile_features(kuka_nrt_message_handler PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(kuka_nrt_message_handler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  kuka_nrt_message_handler
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  kuka_drivers_core
  kuka_driver_interfaces
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(kuka_nrt_message_handler PRIVATE "KUKA_NRT_MESSAGE_HANDLER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS kuka_nrt_message_handler
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)

endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  kuka_nrt_message_handler
)
ament_export_targets(
  export_${PROJECT_NAME}
)
pluginlib_export_plugin_description_file(controller_interface kuka_nrt_message_handler.xml)
ament_package()
