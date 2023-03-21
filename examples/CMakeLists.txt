project(${CMAKE_PROJECT_NAME})               # Name of project

# Find the python3 package with component NumPy
find_package(Python3 COMPONENTS NumPy)

# Find the header file of Python.h in Python3_INCLUDE_DIRS and numpy/arrayobject.h in include
include_directories(${Python3_INCLUDE_DIRS} include)

add_executable(main main.cpp)           # target

# add_executable(tbot_example tbot_example.cpp)           # target


# Link executable to ILQR library
target_link_libraries(main
    PRIVATE                             # private dependency of the main executable (exectubales are usually private)
    ${Python3_LIBRARIES}
    ALILQGames)

# Link executable to ILQR library
# target_link_libraries(tbot_example
#     PRIVATE                             # private dependency of the main executable (exectubales are usually private)
#     ${Python3_LIBRARIES}
#     ALILQGames)

