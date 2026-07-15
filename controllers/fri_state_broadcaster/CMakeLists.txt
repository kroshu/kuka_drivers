cmake_minimum_required(VERSION 3.5)
project(fri_state_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(kuka_driver_interfaces REQUIRED)
find_package(kuka_drivers_core REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
  fri_state_broadcaster_parameters
  src/fri_state_broadcaster_parameters.yaml
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/fri_state_broadcaster.cpp)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME} controller_interface kuka_driver_interfaces kuka_drivers_core)
target_link_libraries(${PROJECT_NAME} fri_state_broadcaster_parameters)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "FRI_STATE_BROADCASTER_BUILDING_LIBRARY")
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME} fri_state_broadcaster_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

if(BUILD_TESTING)

endif()

ament_package()
