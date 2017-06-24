cmake_minimum_required(VERSION 2.8.3)
project(nautonomous_pose_utm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nautonomous_pose_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs sensor_msgs std_msgs nautonomous_pose_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

