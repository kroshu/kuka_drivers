cmake_minimum_required(VERSION 3.5)
project(xml_handler)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(${PROJECT_NAME} SHARED
  rsi_command_handler.cpp
  xml_element.cpp
  xml_string.cpp
  xml_param.cpp
)

add_executable(decode_test
  decodeTest.cpp)

target_link_libraries(decode_test PUBLIC xml_handler)

install(TARGETS ${PROJECT_NAME} decode_test
  DESTINATION lib/${PROJECT_NAME})

# # EXPORTS
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})