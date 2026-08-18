cmake_minimum_required(VERSION 3.22)
project(robotics VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

option(ROBOTICS_BUILD_TESTS "Build the GoogleTest suite" ON)

add_library(robotics STATIC src/robot.cpp)
add_library(robotics::robotics ALIAS robotics)

target_include_directories(robotics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(robotics PUBLIC cxx_std_20)
target_compile_options(robotics PRIVATE -Wall -Wextra -Wpedantic)

# Sophus >= 1.22 pulls in libfmt for logging unless basic logging is requested.
# Nothing here formats log messages, and Emscripten has no system fmt to link.
target_compile_definitions(robotics PUBLIC SOPHUS_USE_BASIC_LOGGING)

if(EMSCRIPTEN)
    # Eigen and Sophus are header-only. Consuming them as bare include directories
    # keeps the host's CMake packages -- and the native libfmt they may pull in --
    # off the wasm link line. NO_CMAKE_FIND_ROOT_PATH is required because the
    # Emscripten toolchain otherwise confines find_path to its own sysroot.
    find_path(EIGEN3_INCLUDE_DIR
        NAMES Eigen/Dense
        HINTS ENV EIGEN3_ROOT
        PATHS /usr/local/include/eigen3 /usr/include/eigen3 /opt/homebrew/include/eigen3
        NO_CMAKE_FIND_ROOT_PATH
    )
    find_path(SOPHUS_INCLUDE_DIR
        NAMES sophus/se3.hpp
        HINTS ENV SOPHUS_ROOT
        PATHS /usr/local/include /usr/include /opt/homebrew/include
        NO_CMAKE_FIND_ROOT_PATH
    )
    if(NOT EIGEN3_INCLUDE_DIR)
        message(FATAL_ERROR "Eigen 3 headers not found. Set -DEIGEN3_INCLUDE_DIR=<dir containing Eigen/Dense>.")
    endif()
    if(NOT SOPHUS_INCLUDE_DIR)
        message(FATAL_ERROR "Sophus headers not found. Set -DSOPHUS_INCLUDE_DIR=<dir containing sophus/se3.hpp>.")
    endif()
    message(STATUS "Eigen 3 headers: ${EIGEN3_INCLUDE_DIR}")
    message(STATUS "Sophus headers:  ${SOPHUS_INCLUDE_DIR}")

    target_include_directories(robotics SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR} ${SOPHUS_INCLUDE_DIR})

    message(STATUS "Configuring for Emscripten -> WebAssembly")
    add_subdirectory(bindings/wasm)
else()
    find_package(Eigen3 3.4 REQUIRED NO_MODULE)
    find_package(Sophus REQUIRED)
    target_link_libraries(robotics PUBLIC Eigen3::Eigen Sophus::Sophus)

    message(STATUS "Configuring for the native platform")
    add_subdirectory(apps/demo)
    if(ROBOTICS_BUILD_TESTS)
        enable_testing()
        add_subdirectory(tests)
    endif()
endif()
