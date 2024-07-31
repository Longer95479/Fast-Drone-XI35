cmake_minimum_required(VERSION 3.0.2)
project(lcm_send)

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
 include
  ${catkin_INCLUDE_DIRS}
)
 
add_executable(lcm_send src/lcm_send_node.cpp)
target_link_libraries(lcm_send lcm)
target_link_libraries(lcm_send ${catkin_LIBRARIES})
add_dependencies(lcm_send ${lcm_send_EXPORTED_TARGETS})
