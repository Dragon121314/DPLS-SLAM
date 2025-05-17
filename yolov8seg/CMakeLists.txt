cmake_minimum_required(VERSION 3.10)
PROJECT(yolov8 VERSION 1.0.0 LANGUAGES C CXX CUDA)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -g")
include_directories( "/usr/include/eigen3" ) 



# find CUDA
if(POLICY CMP0146)
  cmake_policy(SET CMP0146 OLD) 
endif()
find_package(CUDA REQUIRED)
list(APPEND ALL_LIBS 
  ${CUDA_LIBRARIES} 
  ${CUDA_cublas_LIBRARY} 
  ${CUDA_nppc_LIBRARY} ${CUDA_nppig_LIBRARY} ${CUDA_nppidei_LIBRARY} ${CUDA_nppial_LIBRARY})
# include cuda's header
list(APPEND INCLUDE_DRIS ${CUDA_INCLUDE_DIRS})

# gather TensorRT lib
set(TensorRT_ROOT /home/robot/ProgramFile/TensorRT-8.5.2.2)
find_library(TRT_NVINFER NAMES nvinfer HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
find_library(TRT_NVINFER_PLUGIN NAMES nvinfer_plugin HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
find_library(TRT_NVONNX_PARSER NAMES nvonnxparser HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
find_library(TRT_NVCAFFE_PARSER NAMES nvcaffe_parser HINTS ${TensorRT_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
find_path(TENSORRT_INCLUDE_DIR NAMES NvInfer.h HINTS ${TensorRT_ROOT} PATH_SUFFIXES include)
list(APPEND ALL_LIBS ${TRT_NVINFER} ${TRT_NVINFER_PLUGIN} ${TRT_NVONNX_PARSER} ${TRT_NVCAFFE_PARSER})
# include tensorrt's headers
list(APPEND INCLUDE_DRIS ${TENSORRT_INCLUDE_DIR})
set(SAMPLES_COMMON_DIR ${TensorRT_ROOT}/samples/common)
list(APPEND INCLUDE_DRIS ${SAMPLES_COMMON_DIR})

find_package(OpenCV 4.2.0 REQUIRED)
include_directories(${INCLUDE_DRIS} ${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)

file(GLOB CPPS 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu
  ${TensorRT_ROOT}/samples/common/logger.cpp 
  ${TensorRT_ROOT}/samples/common/sampleOptions.cpp
  ${TensorRT_ROOT}/samples/common/sampleUtils.cpp
  )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(${PROJECT_NAME} SHARED ${CPPS})

target_link_libraries(${PROJECT_NAME} ${ALL_LIBS} ${OpenCV_LIBRARIES})

set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_SEPARABLE_COMPILATION ON CUDA_ARCHITECTURES 50 61 72 75)
target_compile_options(${PROJECT_NAME} PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:--default-stream per-thread -lineinfo; --use_fast_math --disable-warnings>)
  
# add_executable(app_yolov8 app_yolov8.cpp)
# target_link_libraries(app_yolov8 ${PROJECT_NAME} ${ALL_LIBS} ${OpenCV_LIBRARIES} )

add_executable(app_yolov8_seg app_yolov8_seg.cpp)
target_link_libraries(app_yolov8_seg ${PROJECT_NAME} ${ALL_LIBS} ${OpenCV_LIBRARIES} )


add_executable(mydata mydata.cpp)
target_link_libraries(mydata ${PROJECT_NAME} ${ALL_LIBS} ${OpenCV_LIBRARIES} )

add_executable(duibi duibi.cpp)
target_link_libraries(duibi ${PROJECT_NAME} ${ALL_LIBS} ${OpenCV_LIBRARIES} )







