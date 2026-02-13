cmake_minimum_required(VERSION 3.8)
project(kuka_external_axis_examples)

find_package(ament_cmake REQUIRED)
find_package(kuka_kl_support REQUIRED)
find_package(kuka_agilus_support REQUIRED)

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/test_examples.py)
endif()

ament_package()
