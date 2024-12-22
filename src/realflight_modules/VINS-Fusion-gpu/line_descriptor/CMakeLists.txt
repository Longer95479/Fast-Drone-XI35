cmake_minimum_required(VERSION 3.0.2)
project(line_descriptor)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3 -mtune=native -march=native")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

set(OpenCV_DIR /usr/local/opencv-4.5.4/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES line_descriptor
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

list(APPEND LINK_LIBS ${OpenCV_LIBS} )

add_library(line_descriptor 
           src/binary_descriptor_custom.cpp
           src/binary_descriptor_matcher.cpp
           src/draw_custom.cpp
           src/LSDDetector_custom.cpp)
           
target_link_libraries( line_descriptor ${LINK_LIBS} )
