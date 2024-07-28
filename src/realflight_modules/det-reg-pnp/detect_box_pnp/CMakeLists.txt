cmake_minimum_required(VERSION 3.0.2)
project(detect_box_pnp)

#set(cv_bridge_DIR ~/cv_bridge/devel/share/cv_bridge/cmake)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rostime
  roscpp
  rosconsole
  roscpp_serialization
  sensor_msgs
  image_transport
  cv_bridge_3416
  actionlib
  tf2_eigen
  tf
  geometry_msgs
  actionlib_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
)

add_message_files(
  FILES
  DetectInfo.msg
)

add_service_files(
  FILES
  ProcessImage.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs 
  sensor_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS
    thread
    date_time
)

# find_package(OpenCV REQUIRED
#                   NO_MODULE
#                   PATHS /usr/local/opencv3.4.16 
#                   NO_DEFAULT_PATH)
#include("~/pnp_det_package/opencv-3.4.16/build/OpenCVConfig.cmake")
include("${CMAKE_SOURCE_DIR}/realflight_modules/det-reg-pnp/opencv3416_aruco_modified_install/share/OpenCV/OpenCVConfig.cmake")
find_package(Eigen3 REQUIRED)

add_definitions(-DOPENCV)
add_definitions(-O4 -g)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INUCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_library(bottom_camera_lib src/bottom_camera.cpp)
target_link_libraries(bottom_camera_lib ${OpenCV_LIBS})

add_executable(detect_box_pnp
    src/detect_box_pnp_node.cpp
)

target_link_libraries(detect_box_pnp bottom_camera_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)
