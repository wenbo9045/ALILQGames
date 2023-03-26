project(${CMAKE_PROJECT_NAME})               # Name of project

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_library(ALILQGames NPlayerModel.cpp ALILQGames.cpp UseImGui.cpp AL.cpp)              # All the cpp files part of the alqp library by default, it's a STATIC lib 

target_link_libraries (ALILQGames Eigen3::Eigen imgui glfw glad)

# target_compile_options (ALILQGames PRIVATE -Ofast -O3)

# Where to find the header files
message(CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

target_include_directories(ALILQGames
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    )