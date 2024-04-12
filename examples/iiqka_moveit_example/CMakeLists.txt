cmake_minimum_required(VERSION 3.5)
project(iiqka_moveit_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

include_directories(include)

add_executable(moveit_basic_planners_example src/moveit_basic_planners_example.cpp)
ament_target_dependencies(moveit_basic_planners_example
  moveit_ros_planning_interface
  moveit_visual_tools
)

add_executable(moveit_collision_avoidance_example src/moveit_collision_avoidance_example.cpp)
ament_target_dependencies(moveit_collision_avoidance_example
  moveit_ros_planning_interface
  moveit_visual_tools
)

add_executable(moveit_constrained_planning_example src/moveit_constrained_planning_example.cpp)
ament_target_dependencies(moveit_constrained_planning_example
  moveit_ros_planning_interface
  moveit_visual_tools
)

add_executable(moveit_depalletizing_example src/moveit_depalletizing_example.cpp)
ament_target_dependencies(moveit_depalletizing_example
  moveit_ros_planning_interface
  moveit_visual_tools
)

install(TARGETS
  moveit_basic_planners_example
  moveit_collision_avoidance_example
  moveit_constrained_planning_example
  moveit_depalletizing_example
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)

endif()

ament_package()
