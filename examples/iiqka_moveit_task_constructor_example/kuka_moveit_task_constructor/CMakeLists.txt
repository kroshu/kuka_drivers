cmake_minimum_required(VERSION 3.8)
project(kuka_moveit_task_constructor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(kuka_moveit_task_constructor src/kuka_moveit_task_constructor.cpp)
ament_target_dependencies(kuka_moveit_task_constructor moveit_task_constructor_core rclcpp)
target_include_directories(kuka_moveit_task_constructor PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(kuka_moveit_task_constructor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(TARGETS kuka_moveit_task_constructor
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