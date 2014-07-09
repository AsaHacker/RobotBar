cmake_minimum_required(VERSION 2.8.3)
project(robot_bar_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################



###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_bar_pkg
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple src/unreleased/simple.cc)

target_link_libraries(simple
  ${catkin_LIBRARIES}  
)

#############
## Install ##
#############


#############
## Testing ##
#############

