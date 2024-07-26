cmake_minimum_required(VERSION 3.0.2)
project(target_recognition)

#set(cv_bridge_DIR ~/cv_bridge/devel/share/cv_bridge/cmake)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge_3416
)

add_service_files(
  FILES
  ProcessImage.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/target_recognition_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

