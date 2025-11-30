cmake_minimum_required(VERSION 3.0.2)
project(motion_capture)

set(CMAKE_VERBOSE_MAKEFILE "false")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/motion_capture_node.cpp
  src/Process.cpp
  src/Parameters.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
