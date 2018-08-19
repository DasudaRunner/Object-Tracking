# cmake needs this line
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -DNDEBUG -pthread -fopenmp -pipe")
add_definitions(-std=c++11)
# Define project name
project(demo)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# Declare the executable target built from your sources
add_executable(demo
        demo.cpp
        ../tracking.hpp
        )
# Link your application with OpenCV libraries
target_link_libraries(demo ${OpenCV_LIBS})
