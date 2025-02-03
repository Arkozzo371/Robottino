cmake_minimum_required(VERSION 3.0.2)
project(robottino)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(

  catkin REQUIRED COMPONENTS

# Utitity pkgs
  roscpp
  rospy
  tf2
  tf2_ros
  dynamic_reconfigure

# Message pkgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  
)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages

add_message_files(
  
  FILES

  Wheel_speeds_message.msg

)

## Generate services

add_service_files(

  FILES
 
  Pose_Reset.srv

)

## Generate added messages and services with any dependencies listed here

generate_messages(

  DEPENDENCIES

  std_msgs
  geometry_msgs
  sensor_msgs

)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/Integration_parameters.cfg
  cfg/robot_parameters.cfg
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robottino
  CATKIN_DEPENDS message_runtime
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs sensor_msgs message_runtime
#  DEPENDS system_lib

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include ${catkin_INCLUDE_DIRS})


## Velocities


## Add library for Velocities class

add_library(velocities_class src/Velocities.cpp)
add_dependencies(velocities_class ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocities_class ${catkin_LIBRARIES})

# main_Velocities

add_executable(Velocities_complete src/main_Velocities.cpp)
add_dependencies(Velocities_complete ${catkin_EXPORTED_TARGETS})
target_link_libraries(Velocities_complete ${catkin_LIBRARIES} velocities_class)

## Odometry

## Add library for Odometry class
add_library(odometry_class src/Odometry.cpp)
add_dependencies(odometry_class ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry_class ${catkin_LIBRARIES})

## Add executable files with its dependences and libraries

# main_Odometry
add_executable(Odometry_complete src/main_Odometry.cpp)
add_dependencies(Odometry_complete ${catkin_EXPORTED_TARGETS})
target_link_libraries(Odometry_complete ${catkin_LIBRARIES} odometry_class)

## Wheel_speeds

## Add library for Odometry class
add_library(wheel_speeds_class src/Wheel_speeds.cpp)
add_dependencies(wheel_speeds_class robottino_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheel_speeds_class ${catkin_LIBRARIES})

## Add executable files with its dependences and libraries

# main_Wheel_speeds
add_executable(Wheel_speeds_complete src/main_Wheel_speeds.cpp)
add_dependencies(Wheel_speeds_complete ${catkin_EXPORTED_TARGETS})
target_link_libraries(Wheel_speeds_complete ${catkin_LIBRARIES} wheel_speeds_class)
