cmake_minimum_required(VERSION 3.0.2)
project(bt)

find_package(catkin REQUIRED COMPONENTS
    tf
  nav_msgs
  geometry_msgs
  message_generation
  actionlib
  roscpp
  roslib
  fsm
  bspline_race
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES bt
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
    inc
    inc/exported 
    inc/exported/behaviac_generated/types/internal
  ${catkin_INCLUDE_DIRS}
)


set(BOOST_THREAD_LIBRARY boost_thread -pthread)

aux_source_directory(src/.  SRC_LIST1)
aux_source_directory(inc/exported/behaviac_generated/types/internal  SRC_LIST2)

 add_executable(bt ${SRC_LIST1} ${SRC_LIST2})
 target_link_libraries(bt pthread ${catkin_LIBRARIES}
 ${CMAKE_CURRENT_SOURCE_DIR}/libs/libbehaviac_gcc_release.a
libpthread.so)

