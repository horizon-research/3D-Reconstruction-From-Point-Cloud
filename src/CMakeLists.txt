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
find_package(OpenCV REQUIRED)

# Add make cleanup
add_custom_target(cleanup
   COMMAND ${CMAKE_COMMAND} -P cleanup.cmake
)

# Add Executable
add_executable(pointsTransfer pointsTransfer.cpp)

# Custom Option for Multi-threading
option(MULTI_THREADING OFF)

# Custom Macros for Multi-threading
if(MULTI_THREADING)
  add_definitions(-DMULTI_THREADING)
  # Find OpenMP
  find_package(OpenMP)
  # Link OpenMP
  target_link_libraries(pointsTransfer OpenMP::OpenMP_CXX)
endif()

# Link libraries
target_link_libraries(pointsTransfer CGAL::CGAL ${OpenCV_LIBS})



