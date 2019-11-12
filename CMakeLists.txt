cmake_minimum_required(VERSION 3.5)
project(kuka_sunrise_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_library(libFRIClient.a
	PATHS /home/rosdeveloper/Documents/fri_client_source/lib)

include_directories(include)

add_library(robot_manager SHARED 
	src/robot_manager.cpp
	src/tcp_connection.cpp)
	
add_library(robot_control_client SHARED
	src/robot_control_client.cpp
	src/robot_observer.cpp
	src/robot_commander.cpp)
ament_target_dependencies(robot_control_client rclcpp rclcpp_lifecycle std_msgs sensor_msgs)

add_executable(robot_manager_node
	src/robot_manager_node.cpp)
ament_target_dependencies(robot_manager_node rclcpp rclcpp_lifecycle)
target_link_libraries(robot_manager_node
	robot_manager)
	
add_executable(robot_control_node
	src/robot_control_node.cpp)
ament_target_dependencies(robot_control_node rclcpp rclcpp_lifecycle std_msgs sensor_msgs)
target_link_libraries(robot_control_node 
	robot_control_client
	/home/rosdeveloper/Documents/fri_client_source/lib/libFRIClient.a)

	
install(TARGETS robot_manager robot_control_client robot_manager_node robot_control_node
	 DESTINATION lib/${PROJECT_NAME})
	
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
