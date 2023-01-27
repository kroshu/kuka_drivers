cmake_minimum_required(VERSION 3.5)
project(kuka_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(kuka_driver_interfaces REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/timing_controller.cpp
  src/robot_state_broadcaster.cpp
  src/joint_impedance_controller.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME} rclcpp controller_interface std_msgs kuka_driver_interfaces
)

pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
