cmake_minimum_required(VERSION 3.0.2)
project(target_merge)
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  rospy
  sensor_msgs
  std_msgs
  search_plan
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  TargetMerged_Message.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )


# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES target_merge_lib
   CATKIN_DEPENDS geometry_msgs message_generation message_runtime roscpp rospy sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(target_merge_lib src/target_merge.cpp)
target_link_libraries(target_merge_lib ${catkin_LIBRARIES})

add_executable(target_merge_node src/target_merge_node.cpp)
target_link_libraries(target_merge_node target_merge_lib)
add_dependencies(target_merge_node search_plan_generate_messages_cpp)

add_dependencies(target_merge_lib 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)