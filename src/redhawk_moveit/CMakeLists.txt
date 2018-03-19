cmake_minimum_required(VERSION 2.8.3)
project(redhawk_moveit)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools
    message_generation
    actionlib_msgs
    actionlib
    
)

find_package(Boost REQUIRED COMPONENTS
    system 
    filesystem 
    date_time 
    thread)
	
#add_action_files(DIRECTORY action FILES fibo.action)
#add_message_files(DIRECTORY msg FILES redhawk_cmd.msg)
#generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    message_runtime
    roscpp
    rospy
    std_msgs
    actionlib_msgs
    actionlib
    
  DEPENDS
    EIGEN3
    Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



###########
## Build ##
###########

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
