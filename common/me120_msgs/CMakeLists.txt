cmake_minimum_required(VERSION 2.8.3)
project(me120_msgs)

find_package(catkin REQUIRED COMPONENTS 
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  jsk_recognition_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  gpsPacket.msg
  CloudCluster.msg
  CloudClusterArray.msg
  img_obj.msg
  img_objs.msg
  PcSegConfig.msg
  PointsImage.msg
  StartInfo.msg
  ClusterMember.msg
  ClusterMemberArray.msg
)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs jsk_recognition_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)
