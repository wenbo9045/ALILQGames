cmake_minimum_required(VERSION 3.10)    # Minimum version of CMake you want to use
set(CMAKE_CXX_STANDARD 17)               
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(ALILQGames VERSION 1.0)               # Name of project

#find_package (Eigen3 3.3 REQUIRED NO_MODULE)


if(NOT CMAKE_BUILD_TYPE)
  # set(CMAKE_BUILD_TYPE Debug)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenMP)

if (OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "-fopenmp -march=native -fno-math-errno")
#     set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#     set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

if (NOT OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "-fno-math-errno")
endif()

# set(CMAKE_CXX_COM)
# set(CMAKE_CXX_FLAGS "-march=native")
# set(CMAKE_CXX_FLAGS "-mfma")
# if (MY_COMPILER_OPTION STREQUAL "Intel")
#   MESSAGE(STATUS "** Compiling with Intel settings **")
#   set(CMAKE_CXX_COMPILER "icpc")
# endif()

# add_compile_options(-fPIC)

# set(CMAKE_CXX_FLAGS "-fno-math-errno")


# set(CMAKE_CXX_FLAGS "-Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3")
# set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
# set(CMAKE_CXX_FLAGS_RELEASE "-fopenmp")

add_subdirectory(include)
add_subdirectory(src)

# Find and build third party libraries.
add_subdirectory(thirdpartylibs)

# Add the source directories to the search path so cmake can find our headers.
include_directories(${CMAKE_SOURCE_DIR}/include ${imgui_SOURCE_DIR}/include ${glad_SOURCE_DIR}/include)

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