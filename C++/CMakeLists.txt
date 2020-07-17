# This is the CMake script for compiling this folder.

cmake_minimum_required(VERSION 3.1...3.15)
project( Point_set_processing_3_Examples )



# Find CGAL
find_package(CGAL QUIET)

if ( CGAL_FOUND )

  find_package(Boost QUIET)

  # VisualC++ optimization for applications dealing with large data
  if (MSVC)
    # Quit warning in the lasreader
    add_definitions(-D_CRT_SECURE_NO_DEPRECATE -D_CRT_SECURE_NO_WARNINGS)
    
    if ( CMAKE_SIZEOF_VOID_P EQUAL 4 )
      # Allow Windows 32bit applications to use up to 3GB of RAM
      SET (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LARGEADDRESSAWARE")
    endif()
    # Prints new compilation options
    message( STATUS "USING DEBUG CXXFLAGS   = '${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}'" )
    message( STATUS "USING DEBUG EXEFLAGS   = '${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS_DEBUG}'" )
    message( STATUS "USING RELEASE CXXFLAGS = '${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}'" )
    message( STATUS "USING RELEASE EXEFLAGS = '${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS_RELEASE}'" )
  endif()

  # Activate concurrency?
  option(CGAL_ACTIVATE_CONCURRENT_PSP3
         "Enable concurrency"
         OFF)
     
  if( CGAL_ACTIVATE_CONCURRENT_PSP3 OR ENV{CGAL_ACTIVATE_CONCURRENT_PSP3} )
    find_package( TBB REQUIRED )
  endif()
    
  
  # Use Eigen
  find_package(Eigen3 3.1.0) #(requires 3.1.0 or greater)
  if (EIGEN3_FOUND)
    include( ${EIGEN3_USE_FILE} )
    # Executables that require Eigen
    create_single_source_cgal_program( "pointsTransfer.cpp" )
  else()

    message(STATUS "NOTICE: Some of the executables in this directory need Eigen 3.1 (or greater) and will not be compiled.")
  endif()

  foreach(target
      normal_estimation)
    if(TBB_FOUND AND TARGET ${target})
      CGAL_target_use_TBB(${target})
    endif()
  endforeach()

else()
    message(STATUS "NOTICE: This program requires the CGAL library, and will not be compiled.")
endif()
