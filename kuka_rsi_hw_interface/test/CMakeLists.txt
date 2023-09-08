find_package(Threads REQUIRED)

set(sources
    src/ut_test_runner.cpp
    src/decode_test.cpp
    src/encode_test.cpp
)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_executable(${PROJECT_NAME}-test ${sources})
target_compile_features(${PROJECT_NAME}-test PUBLIC cxx_std_17 PRIVATE cxx_std_17)

target_link_libraries(${PROJECT_NAME}-test
    ${SYSTEMD_LIBRARIES}
    gtest
    gtest_main
    xml_handler
    ${PROJECT_NAME}
    ${CMAKE_THREAD_LIBS_INIT} # it should be last as lpthread should be last in linking invocation...
)

target_include_directories(${PROJECT_NAME}-test PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/test/include>
)

add_test(NAME ${PROJECT_NAME}-test COMMAND ${PROJECT_NAME}-test)