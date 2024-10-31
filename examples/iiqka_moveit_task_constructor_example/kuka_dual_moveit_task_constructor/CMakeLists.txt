cmake_minimum_required(VERSION 3.8)
project(kuka_dual_moveit_task_constructor_depalletizing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(kuka_dual_moveit_task_constructor_depalletizing src/mtc_depalletizing_task_node.cpp)
ament_target_dependencies(kuka_dual_moveit_task_constructor_depalletizing moveit_task_constructor_core rclcpp)
target_include_directories(kuka_dual_moveit_task_constructor_depalletizing PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(kuka_dual_moveit_task_constructor_depalletizing PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(TARGETS kuka_dual_moveit_task_constructor_depalletizing
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)

endif()

ament_package()
