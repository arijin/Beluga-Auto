cmake_minimum_required(VERSION 2.8.3)
project(me120_tf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  turtlesim
  std_msgs
  sensor_msgs
  geometry_msgs
  me120_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES learning_tf
#  CATKIN_DEPENDS roscpp rospy tf turtlesim
#  DEPENDS system_lib
    CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    me120_msgs
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/tf_broadcaster_me120.cpp
)
add_dependencies(${PROJECT_NAME}_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(tf_echo_node
  src/tf_echo.cpp
)
add_dependencies(tf_echo_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tf_echo_node
  ${catkin_LIBRARIES}
)

