cmake_minimum_required(VERSION 3.0.2)
project(lcm_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(FILES CollaSignal.msg TakeoffLand.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lcm_node src/lcm_node.cpp)
target_link_libraries(lcm_node lcm)
target_link_libraries(lcm_node ${catkin_LIBRARIES})
add_dependencies(lcm_node ${lcm_node_EXPORTED_TARGETS})

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node lcm)
target_link_libraries(test_node ${catkin_LIBRARIES})
add_dependencies(test_node ${lcm_node_EXPORTED_TARGETS})

add_executable(sent_takeoff_node src/sent_takeoff.cpp)
target_link_libraries(sent_takeoff_node lcm)
target_link_libraries(sent_takeoff_node ${catkin_LIBRARIES})
add_dependencies(sent_takeoff_node ${lcm_node_EXPORTED_TARGETS})
