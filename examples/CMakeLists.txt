cmake_minimum_required(VERSION 2.8)
PROJECT(ydlidar_compensator_component)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-std=c++11) # Use C++11

#Include directories
include_directories(${CMAKE_SOURCE_DIR})
############## YDLIDAR SDK START#####################################
#find ydlidar_sdk package
find_package(ydlidar_sdk REQUIRED)
#Include directories
include_directories(
  ${YDLIDAR_SDK_INCLUDE_DIRS}
)
#############################################################################
FILE(GLOB SRCS "*.cpp")
FILE(GLOB HDRS "*.h")

#link library directories
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

#add executable
add_executable(${PROJECT_NAME} ${HDRS} ${SRCS})

#Link project to ydlidar_sdk library.
target_link_libraries(${PROJECT_NAME} ydlidar_compensator ${YDLIDAR_SDK_LIBRARIES})

############## YDLIDAR SDK END#####################################
