cmake_minimum_required(VERSION 3.0.2)
project(teleop_custom)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)


################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package()

## Build nodes
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ps3_twist src/ps3_twist.cpp)
target_link_libraries(ps3_twist ${catkin_LIBRARIES})
add_dependencies(ps3_twist teleop_custom_generate_messages_cpp)

add_executable(mouse_twist src/mouse_twist.cpp)
target_link_libraries(mouse_twist ${catkin_LIBRARIES})
add_dependencies(mouse_twist teleop_custom_generate_messages_cpp)

add_executable(tech_mobility_X638V_twist src/tech_mobility_X638V_twist.cpp)
target_link_libraries(tech_mobility_X638V_twist ${catkin_LIBRARIES})
add_dependencies(tech_mobility_X638V_twist teleop_custom_generate_messages_cpp)
