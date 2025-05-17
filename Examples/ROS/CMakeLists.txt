cmake_minimum_required(VERSION 3.10)
project(orbslam3)
 
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -O3 -g -pthread")


# LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake_modules)
#rosrun orbslam3 build/RGBD /home/robot/ORB_SLAM/ORB_SLAM3_detailed_comments-master/Vocabulary/ORBvoc.txt /home/robot/ORB_SLAM/ORB_SLAM3_detailed_comments-master/Examples/RGB-D/RealSense_D435i.yaml 
# 要保证整个工程的opencv版本一致，包括dbow，源码以及ros相关的
# 3 4 都可以正常运行
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(GLEW REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
catkin_package(CATKIN_DEPENDS cv_bridge geometry_msgs image_transport nav_msgs rosbag roscpp rospy sensor_msgs std_msgs tf)


# #find CUDA
# if(POLICY CMP0146)
#   cmake_policy(SET CMP0146 OLD) 
# endif()
# find_package(CUDA REQUIRED)
# list(APPEND ALL_LIBS 
#   ${CUDA_LIBRARIES} 
#   ${CUDA_cublas_LIBRARY} 
#   ${CUDA_nppc_LIBRARY} ${CUDA_nppig_LIBRARY} ${CUDA_nppidei_LIBRARY} ${CUDA_nppial_LIBRARY})
# # include cuda's header
# list(APPEND INCLUDE_DRIS ${CUDA_INCLUDE_DIRS})
# set(TensorRT_ROOT /home/robot/ProgramFile/TensorRT-8.5.2.2)
# find_library(TRT_NVINFER NAMES nvinfer HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
# find_library(TRT_NVINFER_PLUGIN NAMES nvinfer_plugin HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
# find_library(TRT_NVONNX_PARSER NAMES nvonnxparser HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
# find_library(TRT_NVCAFFE_PARSER NAMES nvcaffe_parser HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
# find_path(TENSORRT_INCLUDE_DIR NAMES NvInfer.h HINTS ${TensorRT_ROOT} PATH_SUFFIXES include)
# list(APPEND ALL_LIBS ${TRT_NVINFER} ${TRT_NVINFER_PLUGIN} ${TRT_NVONNX_PARSER} ${TRT_NVCAFFE_PARSER})
# # include tensorrt's headers
# list(APPEND INCLUDE_DRIS ${TENSORRT_INCLUDE_DIR})
# set(SAMPLES_COMMON_DIR ${TensorRT_ROOT}/samples/common)
# list(APPEND INCLUDE_DRIS ${SAMPLES_COMMON_DIR})



include_directories(
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../
${PROJECT_SOURCE_DIR}/../../include
${PROJECT_SOURCE_DIR}/../../include/CameraModels
${PROJECT_SOURCE_DIR}/../../Thirdparty/Sophus
${GLEW_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
/home/robot/ProgramFile/TensorRT-8.5.2.2/include
/home/robot/ProgramFile/TensorRT-8.5.2.2/samples/common
/usr/local/cuda-11.8/targets/x86_64-linux/include
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../lib/libORB_SLAM3.so
${OPENGL_LIBRARIES} 
${GLEW_LIBRARIES}
-lboost_system
)



# set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_SEPARABLE_COMPILATION ON CUDA_ARCHITECTURES 50 61 72 75)
# target_compile_options(${PROJECT_NAME} PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:--default-stream per-thread -lineinfo; --use_fast_math --disable-warnings>)
  


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

# add_executable(Mono src/ros_mono.cc)
# target_link_libraries(Mono ${catkin_LIBRARIES} ${LIBS})

# Node for monocular camera (Augmented Reality Demo)
# 由于里面代码没有将cv::mat 改成 SOPUS的暂时注释掉，不然会报错
# rosbuild_add_executable(MonoAR
# src/AR/ros_mono_ar.cc
# src/AR/ViewerAR.h
# src/AR/ViewerAR.cc
# )
# target_link_libraries(MonoAR ${LIBS} )

# add_executable(stereo src/ros_stereo.cc)
# target_link_libraries(stereo ${catkin_LIBRARIES} ${LIBS})

add_executable(RGBD src/ros_rgbd.cc)
target_link_libraries(RGBD ${catkin_LIBRARIES} ${LIBS} pthread)

# add_executable(Mono_Inertial src/ros_mono_inertial.cc)
# target_link_libraries(Mono_Inertial ${catkin_LIBRARIES} ${LIBS})

# add_executable(stereo_inertial src/ros_stereo_inertial.cc)
# target_link_libraries(stereo_inertial ${catkin_LIBRARIES} ${LIBS})
