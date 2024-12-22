cmake_minimum_required(VERSION 3.0.2)
project(thread_pool)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES thread_pool
  CATKIN_DEPENDS roscpp rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(thread_pool src/thread_pool.cpp)
target_link_libraries(thread_pool ${catkin_LIBRARIES})

