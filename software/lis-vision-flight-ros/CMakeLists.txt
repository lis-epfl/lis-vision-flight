cmake_minimum_required(VERSION 3.0.2)
project(lis-vision-flight)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  rospy
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lis-vision-flight
#  CATKIN_DEPENDS geometry_msgs mavros_msgs rospy sensor_msgs std_msgs whycon
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
