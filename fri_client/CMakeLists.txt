cmake_minimum_required(VERSION 3.5)
project(fri)

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

find_package(ament_cmake REQUIRED)
find_package(nanopb REQUIRED)

### Append CMAKE_MODULE_PATH by custom find file
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

include_directories(include src)

### Add the Fast Robot Interface libray
add_library(fri_client SHARED
    src/HWIFClientApplication.cpp
    src/friClientApplication.cpp
    src/friCommandMessageEncoder.cpp
    src/friLBRClient.cpp
    src/friLBRCommand.cpp
    src/friLBRState.cpp
    src/FRIMessages.pb.c
    src/friMonitoringMessageDecoder.cpp
    src/friTransformationClient.cpp
    src/friUdpConnection.cpp
    src/pb_frimessages_callbacks.c
)


file(GLOB private_headers
  LIST_DIRECTORIES FALSE
  RELATIVE "${PROJECT_SOURCE_DIR}"
  src/friClientData.h
  src/friCommandMessageEncoder.h
  src/FRIMessages.pb.h
  src/friMonitoringMessageDecoder.h
  src/pb_frimessages_callbacks.h
  )

target_link_libraries(fri_client PRIVATE protobuf-nanopb)

ament_export_targets(fri_client HAS_LIBRARY_TARGET)

install(DIRECTORY include/fri DESTINATION include)
install(FILES ${private_headers} DESTINATION include)

install(
  TARGETS fri_client
  EXPORT fri_client
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_package()
