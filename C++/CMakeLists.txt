# CMake script for compiling this project.

cmake_minimum_required(VERSION 3.1...3.15)
project(pointsTransfer)

# Use Release build type for performance
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif()

# Find CGAL
find_package(CGAL REQUIRED)

# Add make clean
add_custom_target(cleanup
   COMMAND ${CMAKE_COMMAND} -P cleanup.cmake
)

# Add Executable
add_executable(pointsTransfer pointsTransfer.cpp)
target_link_libraries(pointsTransfer CGAL::CGAL)