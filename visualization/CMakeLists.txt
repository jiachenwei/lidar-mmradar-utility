cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(SOURCE_FILE lriovis.cpp lriovis.hpp)

add_library(visualization ${SOURCE_FILE})

find_package(OpenCV 3.4.14 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
                    ${Eigen3_INCLUDE_DIRS})