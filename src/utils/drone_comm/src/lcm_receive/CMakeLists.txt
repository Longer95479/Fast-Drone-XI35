cmake_minimum_required(VERSION 3.0.2)
project(lcm_receive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  TargetMerged_Message.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lcm_receive src/lcm_receive_node.cpp)
target_link_libraries(lcm_receive lcm)
target_link_libraries(lcm_receive ${catkin_LIBRARIES})
add_dependencies(lcm_receive ${lcm_receive_EXPORTED_TARGETS})
