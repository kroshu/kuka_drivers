cmake_minimum_required(VERSION 3.5)

project(ext_axis_examples_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
