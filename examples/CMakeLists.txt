project(${CMAKE_PROJECT_NAME})               # Name of project

# Find the python3 package with component NumPy
find_package(Python3 COMPONENTS Interpreter Development NumPy)
find_package(pybind11 REQUIRED)
find_package(matplotlibcpp17 REQUIRED)

# Find the header file of Python.h in Python3_INCLUDE_DIRS and numpy/arrayobject.h in include
include_directories(${Python3_INCLUDE_DIRS} include)

message(Python3_INCLUDE_DIRS="${Python3_INCLUDE_DIRS}")

add_executable(main main.cpp)           # target

add_executable(pointmassEx pointmassEx.cpp)

# add_executable(tbot_example tbot_example.cpp)           # target


# Link executable to ILQR library
target_link_libraries(main
    PRIVATE                             # private dependency of the main executable (exectubales are usually private)
    ${Python3_LIBRARIES}
    ALILQGames)

target_link_libraries(pointmassEx
    PRIVATE                             # private dependency of the main executable (exectubales are usually private)
    ${Python3_LIBRARIES}
    pybind11::embed
    matplotlibcpp17::matplotlibcpp17
    ALILQGames)

# Link executable to ILQR library
# target_link_libraries(tbot_example
#     PRIVATE                             # private dependency of the main executable (exectubales are usually private)
#     ${Python3_LIBRARIES}
#     ALILQGames)

