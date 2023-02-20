cmake_minimum_required(VERSION 3.0.2)
project(fsm)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  #message_generatin
  tf
  bspline_race
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(FILES command_acc.msg)
generate_messages(DEPENDENCIES std_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES fsm
#LIBRARIES motordriver
   CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(single_offboard src/single_offboard.cpp)
target_link_libraries(single_offboard ${catkin_LIBRARIES} boost_system boost_filesystem)

add_executable(swarm_cmd src/swarm_cmd.cpp)
target_link_libraries(swarm_cmd ${catkin_LIBRARIES} boost_system boost_filesystem)

add_executable(px4_pos_estimator src/px4_pos_estimator.cpp)
target_link_libraries(px4_pos_estimator ${catkin_LIBRARIES})

