cmake_minimum_required(VERSION 2.8.3)
project(imcoder_reader)

# Set warnings
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall -Wextra -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3 -march=native")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for the package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imcoder_reader	
  CATKIN_DEPENDS roscpp sensor_msgs
)

###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(imcoder_reader
  src/imcoder_reader.cpp
)

## Declare a cpp executable
add_executable(imcoder_reader_node nodes/imcoder_reader_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(imcoder_reader
  ${catkin_LIBRARIES}
  RTIMULib
)

target_link_libraries(imcoder_reader_node
  imcoder_reader
)