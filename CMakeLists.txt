cmake_minimum_required(VERSION 2.8.3)
project(object_experiments)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp
					rospy
					genmsg 
					actionlib_msgs 
					actionlib
					std_msgs
					message_generation
					)


## Generate services in the 'srv' folder
 add_service_files(
   DIRECTORY
   srv 

   FILES
   ChoreographySrv.srv
 )

# Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY 
   action 

   FILES
   Choreography.action
)

## Generate added messages and services
 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
 )


catkin_package(
  CATKIN_DEPENDS message_runtime 
)


## Mark executable scripts (Python etc.) for installation
  install(PROGRAMS
   scripts/move_group_python_interface.py
   scripts/rosbag_recording.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


