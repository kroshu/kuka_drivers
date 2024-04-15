cmake_minimum_required(VERSION 3.5)
project(joint_group_impedance_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(forward_command_controller)
find_package(kuka_drivers_core)
find_package(generate_parameter_library)

include_directories(include)

generate_parameter_library(
  joint_group_impedance_controller_parameters
  src/joint_group_impedance_controller_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/joint_group_impedance_controller.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME} forward_command_controller kuka_drivers_core
)
target_link_libraries(${PROJECT_NAME} joint_group_impedance_controller_parameters)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "JOINT_GROUP_IMPEDANCE_CONTROLLER_BUILDING_LIBRARY")
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

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

if(BUILD_TESTING)

endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
