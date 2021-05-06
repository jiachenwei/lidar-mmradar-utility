cmake_minimum_required(VERSION 3.10)

# set the project name
project(lriovis_demo VERSION 0.1)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(SOURCE_FILE lriovis_demo.cpp)

include_directories( visualization )

link_directories( visualization )

add_subdirectory( visualization )

find_package(OpenCV 3.4.14 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${SOURCE_FILE})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} visualization)
