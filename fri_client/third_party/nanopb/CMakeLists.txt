cmake_minimum_required(VERSION 3.1...3.15)

### CMake support for the nanopb library - protocol buffers for embedded systems
if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

project(NanoPB VERSION 0.2.8
    DESCRIPTION "Nanopb - Protocol Buffers for Embedded Systems."
    LANGUAGES C
)

### Append CMAKE_MODULE_PATH by custom find file
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

### Add the nanopb library
add_library(nanopb
    src/pb_decode.c
    src/pb_encode.c
)

### Add compile options, similar to https://stackoverflow.com/questions/45955272/modern-way-to-set-compiler-flags-in-cross-platform-cmake-project
if(MSVC)
    target_compile_options(nanopb
        PRIVATE
            -DPB_SYSTEM_HEADER=<nanopb/pb_syshdr_win.h>
            -DPB_FIELD_16BIT
            -DWIN32
            -DHAVE_STDINT_H
            -DHAVE_STDBOOL_H
    )
else()
    target_compile_options(nanopb
        PRIVATE
            -O2
            -Wall
            -DHAVE_SOCKLEN_T
            -DPB_SYSTEM_HEADER=<nanopb/pb_syshdr.h>
            -DPB_FIELD_16BIT
            -DHAVE_STDINT_H
            -DHAVE_STDDEF_H
            -DHAVE_STDBOOL_H
            -DHAVE_STDLIB_H
            -DHAVE_STRING_H 
    ) 
endif()

### Create an alias so that one can link against this library inside build tree
add_library(NanoPB::nanopb ALIAS nanopb)

### Set target properties
target_include_directories(nanopb
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

### Install instructions
include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/nanopb)

install(TARGETS nanopb
    EXPORT nanopb-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

### Set exported target's name to NanoPB
set_target_properties(nanopb PROPERTIES EXPORT_NAME NanoPB)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

### Export target to a script
install(EXPORT nanopb-targets
    FILE NanoPBTargets.cmake
    NAMESPACE NanoPB::
    DESTINATION ${INSTALL_CONFIGDIR}
)

### Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/NanoPBConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/NanoPBConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/NanoPBConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

### Install the config, configversion and custom find modules
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/NanoPBConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/NanoPBConfigVersion.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)

### Export target from the build tree
export(EXPORT nanopb-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/NanoPBTargets.cmake NAMESPACE NanoPB)

### Register package in user's package registry
export(PACKAGE NanoPB)