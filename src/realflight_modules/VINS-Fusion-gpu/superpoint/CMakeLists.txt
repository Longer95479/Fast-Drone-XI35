cmake_minimum_required(VERSION 2.8.3)
project(superpoint)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "release")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge_454
  camera_models
  image_transport
)
add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/tensorrtbuffer)

set(OpenCV_DIR /usr/local/opencv-4.5.4/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "    catkin libraries: ${catkin_LIBRARIES}")

find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs image_transport
)

if(EXISTS "/usr/local/include/tensorrt")
    message(STATUS "tensorrt include directory: /usr/local/include/tensorrt")
    link_directories(/usr/local/tensorrt/lib)
    include_directories(/usr/local/include/tensorrt)
else()
    message(STATUS "external path for tensorrt not found")
endif()

cuda_add_library(custom_layernorm SHARED
  trt_plugins/custom_layernorm.cu
)

target_link_libraries(custom_layernorm
  nvinfer
  nvinfer_plugin
)

set(TRT_TARGET_DIR "${CMAKE_CURRENT_SOURCE_DIR}/trt_plugins")
add_custom_command(TARGET custom_layernorm POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:custom_layernorm>
    ${TRT_TARGET_DIR}/
  COMMENT "Copying libcustom_layernorm.so to ${TRT_TARGET_DIR}"
)


include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib SHARED
    src/feature_detector.cpp
    src/feature_tracker.cpp
    src/light_glue.cpp
    src/plnet.cpp
    src/point_matcher.cpp
    src/super_point.cpp
    src/utils.cpp
    src/xfeat.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  nvinfer
  nvonnxparser
  ${OpenCV_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  yaml-cpp
  tensorrtbuffer
  -lboost_serialization
)

add_executable(feature_tracker_node src/feature_tracker_node.cpp)
target_link_libraries(feature_tracker_node ${PROJECT_NAME}_lib ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/feature_match_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
