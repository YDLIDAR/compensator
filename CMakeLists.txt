cmake_minimum_required(VERSION 2.8)
PROJECT(ydlidar_compensator)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-std=c++11) # Use C++11
#Include directories
include_directories(
     ${CMAKE_SOURCE_DIR}
)
FILE(GLOB SRCS "*.cpp")
FILE(GLOB HDRS "*.h")
FILE(GLOB MATRIX_HDRS "matrix/*.hpp")

add_subdirectory(examples)

#add library
add_library(${PROJECT_NAME} ${HDRS} ${SRCS} ${MATRIX_HDRS})


