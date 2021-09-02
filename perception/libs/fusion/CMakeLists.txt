cmake_minimum_required (VERSION 3.5)
project(fusion_lib)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# set(CMAKE_CXX_STANDARD 11)
add_definitions(-std=c++11)

## CUDA setting
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(yaml-cpp REQUIRED)
find_package(CUDA REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # ROS core
  roscpp
  # ROS messages
  sensor_msgs
  std_msgs
  geometry_msgs
  jsk_recognition_msgs
  me120_msgs
  cv_bridge
  image_transport
  message_filters
  # ROS PCL
  pcl_conversions
  pcl_ros
  # 3rd modules for ROS, like Eigen
  cmake_modules
  # local packages
  common_lib
  object_builders_lib
)

## System dependencies are found with CMake's conventions
### PCL
find_package(PCL 1.8 REQUIRED COMPONENTS
  common
  io
  filters
  features
  segmentation
)
# set(PCL_INCLUDE_DIRS /usr/include/pcl-1.8)  # 根据实际地址修改
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
### Eigen
find_package(Eigen3 REQUIRED)
### OpenCV
find_package(OpenCV REQUIRED) 
link_directories(${OpenCV_LIBRARY_DIRS})
### CUDA
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
### tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)

catkin_package(
  CATKIN_DEPENDS object_builders_lib segmenters_lib
  #  DEPENDS system_lib
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
cuda_add_library(myplugins SHARED 
  ${PROJECT_SOURCE_DIR}/src/yololayer.cu
)
target_link_libraries(myplugins
  nvinfer 
  cudart
)

add_library(${PROJECT_NAME}
  src/yolov5_d.cpp
  src/image_trans.cpp
  src/fusion.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  nvinfer
  cudart
  myplugins
 )

 #############
## Install ##
#############

# ...


#############
## Samples ##
#############

## Comment following line to avoid generating demo node
add_subdirectory(samples)