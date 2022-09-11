cmake_minimum_required(VERSION 3.0.2)
project(rrt_planner)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  gazebo_msgs
  nav_msgs
  sensor_msgs
  message_generation
)

# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  StartGoalPath.srv
  OccMapInfo.srv
  RandomGetPath.srv
  MapOcc.srv
)

generate_messages(  # 自定義 Serivice 中有用到的 ROS library
  DEPENDENCIES
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrt_planner
  CATKIN_DEPENDS 
  roscpp 
  rospy
  geometry_msgs
  gazebo_msgs
  nav_msgs
  message_runtime
  # DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rrt_planner 
  src/rrt_planner.cpp
  src/grid_map.cpp
)
target_link_libraries(rrt_planner  ${catkin_LIBRARIES} )

#############
## Install ##
#############


#############
## Testing ##
#############
