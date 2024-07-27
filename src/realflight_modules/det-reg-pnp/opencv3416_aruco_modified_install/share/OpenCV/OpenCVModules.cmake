# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget opencv_cudev opencv_core opencv_cudaarithm opencv_flann opencv_hdf opencv_imgproc opencv_ml opencv_phase_unwrapping opencv_plot opencv_reg opencv_surface_matching opencv_video opencv_viz opencv_cudabgsegm opencv_cudafilters opencv_cudaimgproc opencv_cudawarping opencv_dnn opencv_features2d opencv_freetype opencv_fuzzy opencv_hfs opencv_img_hash opencv_imgcodecs opencv_line_descriptor opencv_photo opencv_saliency opencv_shape opencv_text opencv_videoio opencv_xphoto opencv_calib3d opencv_cudacodec opencv_cudafeatures2d opencv_cudastereo opencv_datasets opencv_highgui opencv_objdetect opencv_rgbd opencv_stereo opencv_structured_light opencv_tracking opencv_xfeatures2d opencv_ximgproc opencv_xobjdetect opencv_aruco opencv_bgsegm opencv_bioinspired opencv_ccalib opencv_cudalegacy opencv_cudaobjdetect opencv_cudaoptflow opencv_dnn_objdetect opencv_dpm opencv_face opencv_optflow opencv_sfm correspondence multiview numeric simple_pipeline opencv_stitching opencv_superres opencv_videostab)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)


# Compute the installation prefix relative to this file.
get_filename_component(_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
if(_IMPORT_PREFIX STREQUAL "/")
  set(_IMPORT_PREFIX "")
endif()

# Create imported target opencv_cudev
add_library(opencv_cudev SHARED IMPORTED)

# Create imported target opencv_core
add_library(opencv_core SHARED IMPORTED)

set_target_properties(opencv_core PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_cudev"
)

# Create imported target opencv_cudaarithm
add_library(opencv_cudaarithm SHARED IMPORTED)

set_target_properties(opencv_cudaarithm PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudev;opencv_core"
)

# Create imported target opencv_flann
add_library(opencv_flann SHARED IMPORTED)

set_target_properties(opencv_flann PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudev;opencv_core"
)

# Create imported target opencv_hdf
add_library(opencv_hdf SHARED IMPORTED)

set_target_properties(opencv_hdf PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudev;opencv_core"
)

# Create imported target opencv_imgproc
add_library(opencv_imgproc SHARED IMPORTED)

set_target_properties(opencv_imgproc PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudev;opencv_core"
)

# Create imported target opencv_ml
add_library(opencv_ml SHARED IMPORTED)

set_target_properties(opencv_ml PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudev;opencv_core"
)

# Create imported target opencv_phase_unwrapping
add_library(opencv_phase_unwrapping SHARED IMPORTED)

set_target_properties(opencv_phase_unwrapping PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_cudev;opencv_core;opencv_imgproc"
)

# Create imported target opencv_plot
add_library(opencv_plot SHARED IMPORTED)

set_target_properties(opencv_plot PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_cudev;opencv_core;opencv_imgproc"
)

# Create imported target opencv_reg
add_library(opencv_reg SHARED IMPORTED)

set_target_properties(opencv_reg PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_cudev;opencv_core;opencv_imgproc"
)

# Create imported target opencv_surface_matching
add_library(opencv_surface_matching SHARED IMPORTED)

set_target_properties(opencv_surface_matching PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_cudev;opencv_core;opencv_flann"
)

# Create imported target opencv_video
add_library(opencv_video SHARED IMPORTED)

set_target_properties(opencv_video PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_cudev;opencv_core;opencv_imgproc"
)

# Create imported target opencv_viz
add_library(opencv_viz SHARED IMPORTED)

set_target_properties(opencv_viz PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudev;opencv_core"
)

# Create imported target opencv_cudabgsegm
add_library(opencv_cudabgsegm SHARED IMPORTED)

set_target_properties(opencv_cudabgsegm PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_video;opencv_cudev;opencv_core;opencv_imgproc;opencv_video"
)

# Create imported target opencv_cudafilters
add_library(opencv_cudafilters SHARED IMPORTED)

set_target_properties(opencv_cudafilters PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_imgproc;opencv_cudev;opencv_core;opencv_cudaarithm;opencv_imgproc"
)

# Create imported target opencv_cudaimgproc
add_library(opencv_cudaimgproc SHARED IMPORTED)

set_target_properties(opencv_cudaimgproc PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_imgproc;opencv_cudafilters;opencv_cudev;opencv_core;opencv_cudaarithm;opencv_imgproc;opencv_cudafilters"
)

# Create imported target opencv_cudawarping
add_library(opencv_cudawarping SHARED IMPORTED)

set_target_properties(opencv_cudawarping PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_cudev;opencv_core;opencv_imgproc"
)

# Create imported target opencv_dnn
add_library(opencv_dnn SHARED IMPORTED)

set_target_properties(opencv_dnn PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_cudev;opencv_core;opencv_imgproc"
)

# Create imported target opencv_features2d
add_library(opencv_features2d SHARED IMPORTED)

set_target_properties(opencv_features2d PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc"
)

# Create imported target opencv_freetype
add_library(opencv_freetype SHARED IMPORTED)

set_target_properties(opencv_freetype PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_cudev;opencv_core;opencv_imgproc"
)

# Create imported target opencv_fuzzy
add_library(opencv_fuzzy SHARED IMPORTED)

set_target_properties(opencv_fuzzy PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_cudev;opencv_core;opencv_imgproc"
)

# Create imported target opencv_hfs
add_library(opencv_hfs SHARED IMPORTED)

set_target_properties(opencv_hfs PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_cudev;opencv_core;opencv_imgproc"
)

# Create imported target opencv_img_hash
add_library(opencv_img_hash SHARED IMPORTED)

set_target_properties(opencv_img_hash PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_cudev;opencv_core;opencv_imgproc"
)

# Create imported target opencv_imgcodecs
add_library(opencv_imgcodecs SHARED IMPORTED)

set_target_properties(opencv_imgcodecs PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_cudev;opencv_core;opencv_imgproc"
)

# Create imported target opencv_line_descriptor
add_library(opencv_line_descriptor SHARED IMPORTED)

set_target_properties(opencv_line_descriptor PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d"
)

# Create imported target opencv_photo
add_library(opencv_photo SHARED IMPORTED)

set_target_properties(opencv_photo PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_imgproc;opencv_cudafilters;opencv_cudaimgproc;opencv_cudev;opencv_core;opencv_cudaarithm;opencv_imgproc;opencv_cudafilters;opencv_cudaimgproc"
)

# Create imported target opencv_saliency
add_library(opencv_saliency SHARED IMPORTED)

set_target_properties(opencv_saliency PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d"
)

# Create imported target opencv_shape
add_library(opencv_shape SHARED IMPORTED)

set_target_properties(opencv_shape PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_video;opencv_cudev;opencv_core;opencv_imgproc;opencv_video"
)

# Create imported target opencv_text
add_library(opencv_text SHARED IMPORTED)

set_target_properties(opencv_text PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_dnn;opencv_features2d;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_dnn;opencv_features2d"
)

# Create imported target opencv_videoio
add_library(opencv_videoio SHARED IMPORTED)

set_target_properties(opencv_videoio PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_cudev;opencv_core;opencv_imgproc;opencv_imgcodecs"
)

# Create imported target opencv_xphoto
add_library(opencv_xphoto SHARED IMPORTED)

set_target_properties(opencv_xphoto PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_imgproc;opencv_cudafilters;opencv_cudaimgproc;opencv_photo;opencv_cudev;opencv_core;opencv_cudaarithm;opencv_imgproc;opencv_cudafilters;opencv_cudaimgproc;opencv_photo"
)

# Create imported target opencv_calib3d
add_library(opencv_calib3d SHARED IMPORTED)

set_target_properties(opencv_calib3d PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d"
)

# Create imported target opencv_cudacodec
add_library(opencv_cudacodec SHARED IMPORTED)

set_target_properties(opencv_cudacodec PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_cudev;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio"
)

# Create imported target opencv_cudafeatures2d
add_library(opencv_cudafeatures2d SHARED IMPORTED)

set_target_properties(opencv_cudafeatures2d PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_cudafilters;opencv_cudawarping;opencv_features2d;opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_cudafilters;opencv_cudawarping;opencv_features2d"
)

# Create imported target opencv_cudastereo
add_library(opencv_cudastereo SHARED IMPORTED)

set_target_properties(opencv_cudastereo PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d"
)

# Create imported target opencv_datasets
add_library(opencv_datasets SHARED IMPORTED)

set_target_properties(opencv_datasets PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_text;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_text"
)

# Create imported target opencv_highgui
add_library(opencv_highgui SHARED IMPORTED)

set_target_properties(opencv_highgui PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_cudev;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio"
)

# Create imported target opencv_objdetect
add_library(opencv_objdetect SHARED IMPORTED)

set_target_properties(opencv_objdetect PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d"
)

# Create imported target opencv_rgbd
add_library(opencv_rgbd SHARED IMPORTED)

set_target_properties(opencv_rgbd PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d"
)

# Create imported target opencv_stereo
add_library(opencv_stereo SHARED IMPORTED)

set_target_properties(opencv_stereo PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d"
)

# Create imported target opencv_structured_light
add_library(opencv_structured_light SHARED IMPORTED)

set_target_properties(opencv_structured_light PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_phase_unwrapping;opencv_viz;opencv_features2d;opencv_calib3d;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_phase_unwrapping;opencv_viz;opencv_features2d;opencv_calib3d"
)

# Create imported target opencv_tracking
add_library(opencv_tracking SHARED IMPORTED)

set_target_properties(opencv_tracking PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_plot;opencv_video;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_text;opencv_datasets;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_ml;opencv_plot;opencv_video;opencv_dnn;opencv_features2d;opencv_imgcodecs;opencv_text;opencv_datasets"
)

# Create imported target opencv_xfeatures2d
add_library(opencv_xfeatures2d SHARED IMPORTED)

set_target_properties(opencv_xfeatures2d PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_ml;opencv_video;opencv_features2d;opencv_shape;opencv_calib3d;opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_ml;opencv_video;opencv_features2d;opencv_shape;opencv_calib3d"
)

# Create imported target opencv_ximgproc
add_library(opencv_ximgproc SHARED IMPORTED)

set_target_properties(opencv_ximgproc PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_imgcodecs;opencv_calib3d;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_imgcodecs;opencv_calib3d"
)

# Create imported target opencv_xobjdetect
add_library(opencv_xobjdetect SHARED IMPORTED)

set_target_properties(opencv_xobjdetect PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_imgcodecs;opencv_calib3d;opencv_objdetect;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_imgcodecs;opencv_calib3d;opencv_objdetect"
)

# Create imported target opencv_aruco
add_library(opencv_aruco SHARED IMPORTED)

set_target_properties(opencv_aruco PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d"
)

# Create imported target opencv_bgsegm
add_library(opencv_bgsegm SHARED IMPORTED)

set_target_properties(opencv_bgsegm PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_video;opencv_features2d;opencv_calib3d;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_video;opencv_features2d;opencv_calib3d"
)

# Create imported target opencv_bioinspired
add_library(opencv_bioinspired SHARED IMPORTED)

set_target_properties(opencv_bioinspired PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_cudev;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui"
)

# Create imported target opencv_ccalib
add_library(opencv_ccalib SHARED IMPORTED)

set_target_properties(opencv_ccalib PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_imgcodecs;opencv_videoio;opencv_calib3d;opencv_highgui;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_imgcodecs;opencv_videoio;opencv_calib3d;opencv_highgui"
)

# Create imported target opencv_cudalegacy
add_library(opencv_cudalegacy SHARED IMPORTED)

set_target_properties(opencv_cudalegacy PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_video;opencv_cudafilters;opencv_cudaimgproc;opencv_features2d;opencv_calib3d;opencv_objdetect;opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_video;opencv_cudafilters;opencv_cudaimgproc;opencv_features2d;opencv_calib3d;opencv_objdetect"
)

# Create imported target opencv_cudaobjdetect
add_library(opencv_cudaobjdetect SHARED IMPORTED)

set_target_properties(opencv_cudaobjdetect PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_video;opencv_cudafilters;opencv_cudaimgproc;opencv_cudawarping;opencv_features2d;opencv_calib3d;opencv_objdetect;opencv_cudalegacy;opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_video;opencv_cudafilters;opencv_cudaimgproc;opencv_cudawarping;opencv_features2d;opencv_calib3d;opencv_objdetect;opencv_cudalegacy"
)

# Create imported target opencv_cudaoptflow
add_library(opencv_cudaoptflow SHARED IMPORTED)

set_target_properties(opencv_cudaoptflow PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_video;opencv_cudafilters;opencv_cudaimgproc;opencv_cudawarping;opencv_features2d;opencv_calib3d;opencv_objdetect;opencv_cudalegacy;opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_video;opencv_cudafilters;opencv_cudaimgproc;opencv_cudawarping;opencv_features2d;opencv_calib3d;opencv_objdetect;opencv_cudalegacy"
)

# Create imported target opencv_dnn_objdetect
add_library(opencv_dnn_objdetect SHARED IMPORTED)

set_target_properties(opencv_dnn_objdetect PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_imgproc;opencv_dnn;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_cudev;opencv_core;opencv_imgproc;opencv_dnn;opencv_imgcodecs;opencv_videoio;opencv_highgui"
)

# Create imported target opencv_dpm
add_library(opencv_dpm SHARED IMPORTED)

set_target_properties(opencv_dpm PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_imgcodecs;opencv_videoio;opencv_calib3d;opencv_highgui;opencv_objdetect;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_imgcodecs;opencv_videoio;opencv_calib3d;opencv_highgui;opencv_objdetect"
)

# Create imported target opencv_face
add_library(opencv_face SHARED IMPORTED)

set_target_properties(opencv_face PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_video;opencv_cudafilters;opencv_cudaimgproc;opencv_features2d;opencv_photo;opencv_calib3d;opencv_objdetect;opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_video;opencv_cudafilters;opencv_cudaimgproc;opencv_features2d;opencv_photo;opencv_calib3d;opencv_objdetect"
)

# Create imported target opencv_optflow
add_library(opencv_optflow SHARED IMPORTED)

set_target_properties(opencv_optflow PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_video;opencv_features2d;opencv_imgcodecs;opencv_calib3d;opencv_ximgproc;opencv_cudev;opencv_core;opencv_flann;opencv_imgproc;opencv_video;opencv_features2d;opencv_imgcodecs;opencv_calib3d;opencv_ximgproc"
)

# Create imported target opencv_sfm
add_library(opencv_sfm SHARED IMPORTED)

set_target_properties(opencv_sfm PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_ml;opencv_video;opencv_features2d;opencv_imgcodecs;opencv_shape;opencv_calib3d;opencv_xfeatures2d;opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_ml;opencv_video;opencv_features2d;opencv_imgcodecs;opencv_shape;opencv_calib3d;opencv_xfeatures2d"
)

# Create imported target correspondence
add_library(correspondence STATIC IMPORTED)

set_target_properties(correspondence PROPERTIES
  INTERFACE_LINK_LIBRARIES "/usr/local/lib/libglog.so;\$<LINK_ONLY:multiview>;Eigen3::Eigen"
)

# Create imported target multiview
add_library(multiview STATIC IMPORTED)

set_target_properties(multiview PROPERTIES
  INTERFACE_LINK_LIBRARIES "/usr/local/lib/libglog.so;\$<LINK_ONLY:numeric>;Eigen3::Eigen;\$<LINK_ONLY:Ceres::ceres>"
)

# Create imported target numeric
add_library(numeric STATIC IMPORTED)

set_target_properties(numeric PROPERTIES
  INTERFACE_LINK_LIBRARIES "Eigen3::Eigen"
)

# Create imported target simple_pipeline
add_library(simple_pipeline STATIC IMPORTED)

set_target_properties(simple_pipeline PROPERTIES
  INTERFACE_LINK_LIBRARIES "\$<LINK_ONLY:multiview>;\$<LINK_ONLY:Ceres::ceres>"
)

# Create imported target opencv_stitching
add_library(opencv_stitching SHARED IMPORTED)

set_target_properties(opencv_stitching PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_ml;opencv_video;opencv_cudafilters;opencv_cudaimgproc;opencv_cudawarping;opencv_features2d;opencv_shape;opencv_calib3d;opencv_cudafeatures2d;opencv_objdetect;opencv_xfeatures2d;opencv_cudalegacy;opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_ml;opencv_video;opencv_cudafilters;opencv_cudaimgproc;opencv_cudawarping;opencv_features2d;opencv_shape;opencv_calib3d;opencv_cudafeatures2d;opencv_objdetect;opencv_xfeatures2d;opencv_cudalegacy"
)

# Create imported target opencv_superres
add_library(opencv_superres SHARED IMPORTED)

set_target_properties(opencv_superres PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_video;opencv_cudafilters;opencv_cudaimgproc;opencv_cudawarping;opencv_features2d;opencv_imgcodecs;opencv_videoio;opencv_calib3d;opencv_cudacodec;opencv_objdetect;opencv_cudalegacy;opencv_cudaoptflow;opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_video;opencv_cudafilters;opencv_cudaimgproc;opencv_cudawarping;opencv_features2d;opencv_imgcodecs;opencv_videoio;opencv_calib3d;opencv_cudacodec;opencv_objdetect;opencv_cudalegacy;opencv_cudaoptflow"
)

# Create imported target opencv_videostab
add_library(opencv_videostab SHARED IMPORTED)

set_target_properties(opencv_videostab PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_video;opencv_cudafilters;opencv_cudaimgproc;opencv_cudawarping;opencv_features2d;opencv_imgcodecs;opencv_photo;opencv_videoio;opencv_calib3d;opencv_objdetect;opencv_cudalegacy;opencv_cudaoptflow;opencv_cudev;opencv_core;opencv_cudaarithm;opencv_flann;opencv_imgproc;opencv_video;opencv_cudafilters;opencv_cudaimgproc;opencv_cudawarping;opencv_features2d;opencv_imgcodecs;opencv_photo;opencv_videoio;opencv_calib3d;opencv_objdetect;opencv_cudalegacy;opencv_cudaoptflow"
)

if(CMAKE_VERSION VERSION_LESS 2.8.12)
  message(FATAL_ERROR "This file relies on consumers using CMake 2.8.12 or greater.")
endif()

# Load information for each installed configuration.
get_filename_component(_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
file(GLOB CONFIG_FILES "${_DIR}/OpenCVModules-*.cmake")
foreach(f ${CONFIG_FILES})
  include(${f})
endforeach()

# Cleanup temporary variables.
set(_IMPORT_PREFIX)

# Loop over all imported files and verify that they actually exist
foreach(target ${_IMPORT_CHECK_TARGETS} )
  foreach(file ${_IMPORT_CHECK_FILES_FOR_${target}} )
    if(NOT EXISTS "${file}" )
      message(FATAL_ERROR "The imported target \"${target}\" references the file
   \"${file}\"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   \"${CMAKE_CURRENT_LIST_FILE}\"
but not all the files it references.
")
    endif()
  endforeach()
  unset(_IMPORT_CHECK_FILES_FOR_${target})
endforeach()
unset(_IMPORT_CHECK_TARGETS)

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)