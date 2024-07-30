cmake_minimum_required(VERSION 3.0.2)
project(search_plan)
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  nav_msgs
  message_generation
  message_runtime
  quadrotor_msgs
)
find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  SearchService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs
)
include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_library(search_lib src/search_plan_fsm.cpp)
target_link_libraries(search_lib ${catkin_LIBRARIES})

add_executable(search_node src/search_plan_node.cpp)
target_link_libraries(search_node search_lib)

add_dependencies(search_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)