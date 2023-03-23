project(glad CXX C)
cmake_minimum_required(VERSION 3.10)    # Minimum version of CMake you want to use

include_directories(${glad_SOURCE_DIR}/include)
link_directories(${glad_BINARY_DIR})

#find_package(PkgConfig REQUIRED)
#pkg_search_module(GLFW3 REQUIRED glfw)
#include_directories()

file(GLOB_RECURSE glad_srcs ${glad_SOURCE_DIR}/src/*.c*)
add_library(glad ${glad_srcs})


target_link_libraries(glad ${glad_LIBRARIES})

# Where to find the header files
message(CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(glad
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
