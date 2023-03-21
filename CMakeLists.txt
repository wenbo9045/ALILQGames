cmake_minimum_required(VERSION 3.10)    # Minimum version of CMake you want to use
set(CMAKE_CXX_STANDARD 17)               
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(ALILQGames VERSION 1.0)               # Name of project

#find_package (Eigen3 3.3 REQUIRED NO_MODULE)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

add_subdirectory(include)
add_subdirectory(src)

# Option creates a variable and stores an initial value to it unless you overwrite it in cmd line
option(BUILD_EXAMPLES "Build library examples" ON) 
option(BUILD_TESTS "Build library tests" ON)

if (BUILD_EXAMPLES)
    message(STATUS "Building Examples")
    add_subdirectory(examples)
endif() 

if (BUILD_TESTS)
    message(STATUS "Building Tests")
    add_subdirectory(tests)
endif() 