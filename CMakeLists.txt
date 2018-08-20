# cmake needs this line
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -DNDEBUG -pthread -fopenmp -pipe")
add_definitions(-std=c++11)
# Define project name
project(tracking)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}b ${CUDA_INCLUDE_DIRS})

# Declare the executable target built from your sources
add_executable(tracker
        test_camera.cpp
        tracking.hpp
        )
# Link your application with OpenCV libraries
target_link_libraries(tracker dlib ${OpenCV_LIBS} ${CUDA_LIBRARIES})
