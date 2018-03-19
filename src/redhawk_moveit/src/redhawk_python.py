#!/usr/bin/env python2

from __future__ import print_function
import sys
import math
import time
import rospy
import threading
from redhawk_ctl.msg import redhawk_cmd, redhawk_telem
import redhawk_commander

import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion


ROS_RATE = 60


def moveHome(group):
    # For resetting robot in simulation:
    
    print("RETURNING TO HOME POSITION")
    
    home_joints = [0.2,0.5,-0.3,0.2,0.1,0.1]
    home_joints = [0 for i in range(6)]
    #home_joints[1] = 0.2
    
    group.set_joint_value_target(home_joints)
    
    plan = group.plan()
    
    group.execute(plan)

            
def plan(group, target):
    # Set target pose
    group.set_pose_target(position_target)
    #group.set_position_target((position_target.position.x, position_target.position.y, position_target.position.z))

    # Plan
    traj = group.plan()
    
    return traj
        
    
if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('redhawk_moveit_bridge')

    ctl = redhawk_commander.RedhawkController()
    
    r = rospy.Rate(ROS_RATE)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("robot")
    group.set_end_effector_link("link6")
    
    print("============ Reference frame: %s" % group.get_planning_frame())
    print("============ End effector link: %s" % group.get_end_effector_link());
    
    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory)
    
    print("============ Robot Groups:")
    print(robot.get_group_names())
    
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("============")
    
    print("Current pose: ")
    print(group.get_current_pose())
    
    # Reset group
    print("============ Zeroing arm and waiting...")
    ctl.set_pose([0, 0, 0, 0, 0, 0])
    print("============ Zero pose reached. Moving on.")
    #moveHome(group)
    
    print("Home pose: ")
    print(group.get_current_pose())
        
    # Plan trajectory
    current_pose = group.get_current_pose().pose
    position_target = current_pose
    '''
    current_quat = (current_pose.orientation.x,
                    current_pose.orientation.y,
                    current_pose.orientation.z,
                    current_pose.orientation.w)
    current_orient = euler_from_quaternion(current_quat)
    print("Current orientation:", current_orient)
    roll = -math.pi
    pitch = 0
    yaw = math.pi
    quat = quaternion_from_euler(roll, pitch, yaw)
    position_target.orientation.x = quat[0]
    position_target.orientation.y = quat[1]
    position_target.orientation.z = quat[2]
    position_target.orientation.w = quat[3]'''
    position_target.position.z = current_pose.position.z-0.25
    position_target.position.x = current_pose.position.x
    position_target.position.y = current_pose.position.y
    traj = plan(group, position_target)

    print("============ Target pose")
    print(position_target)

    # Execute trajectory in simulation and return home:
    #print("Visualizing trajectory and returning home...")
    #group.execute(traj)
    #print("Reached position: ")
    #print(group.get_current_pose())
    #moveHome(group)
  
    print("Publishing trajectory...")
    ctl.run2(traj.joint_trajectory)
    
    print("Trajectory correctly published! Current pose: ")
    print(group.get_current_pose())
