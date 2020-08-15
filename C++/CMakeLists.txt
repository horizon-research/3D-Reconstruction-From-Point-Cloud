# CMake script for compiling this project.

cmake_minimum_required(VERSION 3.1...3.15)
project(pointsTransfer)

# Use Release build type for performance
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif()

# Find CGAL
find_package(CGAL REQUIRED)

# Find OpenCV


# Add make clean
add_custom_target(cleanup
   COMMAND ${CMAKE_COMMAND} -P cleanup.cmake
)

# Create Program
create_single_source_cgal_program("pointsTransfer.cpp")
