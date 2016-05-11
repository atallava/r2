# CMake root for serial_api
# Version required.
cmake_minimum_required (VERSION 2.8)
# Project name SERIAL_API

project (SERIAL_API)

set (CMAKE_BUILD_TYPE "Debug")

set( LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib" CACHE PATH
    "Target directory for all libraries." )
set( EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin" CACHE PATH
    "Target directory for all executables." )

# Threading and Boost utilities
find_package( Threads REQUIRED )
list( APPEND SERIAL_API_LIBRARIES ${CMAKE_THREAD_LIBS_INIT} )

ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)

find_package( Boost COMPONENTS log REQUIRED )
include_directories( ${Boost_INCLUDE_DIRS} )
list( APPEND SERIAL_API_LIBRARIES ${Boost_LIBRARIES} )
list( APPEND SERIAL_API_LIBRARIES "-lboost_thread" ) # Need to force linking against threads

add_definitions( -Wall -std=c++0x )

set( SERIAL_API_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/include" )
include_directories(${SERIAL_API_INCLUDE_DIRECTORIES})

# Add files here to compile them
add_library(serial_api SHARED 	
  src/Test.cpp
  )

# target_link_libraries(serial_api ${SERIAL_API_LIBRARIES})

# Add files here to make executables out of them
add_executable(run_tests src/run_tests.cpp)
target_link_libraries(run_tests serial_api ${SERIAL_API_LIBRARIES})
