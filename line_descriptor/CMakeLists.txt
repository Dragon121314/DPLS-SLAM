cmake_minimum_required(VERSION 3.20)

project(line_descriptor)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package( OpenCV 4.2.0  REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realsense2  REQUIRED)

include_directories( include/  ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${realsense_INCLUDE_DIR})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/binary_descriptor_matcher.cpp
src/binary_descriptor.cpp
src/bitarray.hpp
src/bitops.hpp
src/draw.cpp
src/LSDDetector.cpp
src/precomp.hpp
src/types.hpp
include/line_descriptor.hpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${realsense2_LIBRARY}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

# add_executable(lsd samples/lsd_lines_extraction.cpp)
# target_link_libraries(lsd ${PROJECT_NAME})

add_executable(line lines_extraction.cpp)
target_link_libraries(line ${PROJECT_NAME})

add_executable(compare_ed compare_ed.cpp)
target_link_libraries(compare_ed ${PROJECT_NAME})

add_executable(kl_line TrackingLineLK.cpp)
target_link_libraries(kl_line ${PROJECT_NAME})

add_executable(twopoint TrackingLineLKTwoPoint.cpp)
target_link_libraries(twopoint ${PROJECT_NAME})


# add_executable(p TrackingPointLK.cpp)
# target_link_libraries(p ${PROJECT_NAME})

add_executable(lsded KLliner.cpp)
target_link_libraries(lsded ${PROJECT_NAME})


add_executable(realsense realsense.cpp)
target_link_libraries(realsense ${PROJECT_NAME})


add_executable(match ExtractAndMatch.cpp)
target_link_libraries(match ${PROJECT_NAME})


add_executable(optim optim.cpp)
target_link_libraries(optim ${PROJECT_NAME})


add_executable(cull cull.cpp)
target_link_libraries(cull ${PROJECT_NAME})

add_executable(q qqq.cpp)
target_link_libraries(q ${PROJECT_NAME}) 

add_executable(KL KL.cpp)
target_link_libraries(KL ${PROJECT_NAME})


















