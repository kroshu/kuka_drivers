cmake_minimum_required(VERSION 3.8)
project(omnimove_external_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

include_directories(include)


## COMPILE
add_library(
  omnimove_external_control
  SHARED
  src/omnimove_external_control.cpp
  src/external_control_message.cpp
)

add_executable(
    platform_move_test
    src/platform_move_test.cpp
)
ament_target_dependencies(
    platform_move_test
    rclcpp std_msgs std_srvs
)

install(TARGETS omnimove_external_control
  DESTINATION lib)

install(TARGETS platform_move_test
  DESTINATION lib/${PROJECT_NAME})


ament_target_dependencies(omnimove_external_control PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
pluginlib_export_plugin_description_file(hardware_interface omnimove_external_control.xml)

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()
ament_export_libraries(
  omnimove_external_control
  ${BOOST_LIBRARIES}
)
