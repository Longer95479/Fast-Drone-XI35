cmake_minimum_required(VERSION 3.0.2)
project(kf_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "release")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)

## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package()

add_library(${PROJECT_NAME}_lib
  src/state_manager.cpp
  src/kf_coordinator.cpp
  src/state_initializer.cpp
  src/kf_interface.cpp
  src/health_monitor.cpp
)
target_link_libraries(${PROJECT_NAME}_lib 
${catkin_LIBRARIES}
yaml-cpp
)

add_executable(${PROJECT_NAME}_node src/kf_nav_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

add_executable(ext_calib_node src/ext_calib_node.cpp src/ext_calib.cpp)
target_link_libraries(ext_calib_node ${catkin_LIBRARIES} yaml-cpp)