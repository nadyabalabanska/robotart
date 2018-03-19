#!/usr/bin/env python2

from __future__ import print_function
import sys
import math
import time
import copy
import dxfgrabber
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import visualization_msgs.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion


DOWNWARD_OFFSET = 0.02
PAINTING_VELOCITY_SCALING_FACTOR = 0.1
GENERAL_VELOCITY_SCALING_FACTOR = 0.3
DEPTH_ADJUST_VELOCITY_SCALING_FACTOR = 0.05


joint_positions_painting_home = [-0.0975, 0.0704, 1.9869, -math.pi, -1.0791, 3.0448]


def read_dxf(filename):
    dxf = dxfgrabber.readfile(filename)
    polylines = [poly for poly in dxf.entities if type(poly) is dxfgrabber.dxfentities.LWPolyline]
    paths = []
    for line in polylines:
        path = []
        for pt in line:
            path += [(pt[0]/1000.0, pt[1]/1000.0)]
        paths += [path]
    return paths


def create_markers(wp, i=0):
    markers_add = visualization_msgs.msg.MarkerArray()
    markers_remove = visualization_msgs.msg.MarkerArray()
    
    marker = visualization_msgs.msg.Marker()
    marker.header.frame_id = "base_link"
    marker.ns = "trajectory_waypoints_{}".format(i)
    marker.id = 0;
    marker.type = 4;
    marker.action = 0;
    for i, p in enumerate(wp):
        marker.points += [p.position]
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    markers_add.markers += [marker]
    marker_remove = copy.deepcopy(marker)
    marker_remove.action = 2
    markers_remove.markers += [marker_remove]

    return markers_add, markers_remove


def move_to_start(group, depth=0, velocity_scale=GENERAL_VELOCITY_SCALING_FACTOR):
    target = geometry_msgs.msg.Pose()
    '''
    current_quat = (current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w)
    current_orient = euler_from_quaternion(current_quat)
    print("Current orientation:", current_orient)'''
    roll = -math.pi # upright: -pi
    pitch = math.pi # upright: 0
    yaw = math.pi   # upright: pi
    quat = quaternion_from_euler(roll, pitch, yaw)
    target.orientation.x = quat[0]
    target.orientation.y = quat[1]
    target.orientation.z = quat[2]
    target.orientation.w = quat[3]
    target.position.z = 0.25 - depth
    target.position.x = 0.35
    target.position.y = -0.21
    group.set_pose_target(target)
    group.set_max_velocity_scaling_factor(velocity_scale)
    plan = group.plan()
    print("======== Moving to starting position")
    group.execute(plan)
    print("Done.")


def plan_cartesian(group, paths):
    current_pose = group.get_current_pose().pose
    num_waypoints = sum([len(path)+2 for path in paths])
    wp = [copy.deepcopy(current_pose) for i in range(num_waypoints)]

    point_count = 0
    for path in paths:
        # Add waypoint at beginning of downward path (extend tool)
        wp[point_count].position.x += path[0][0]
        wp[point_count].position.y += path[0][1]
        point_count += 1

        # Add waypoints for individual path
        for pt in path:
            wp[point_count].position.x += pt[0]
            wp[point_count].position.y += pt[1]
            wp[point_count].position.z -= DOWNWARD_OFFSET
            point_count += 1

        # Add waypoint to retract tool
        wp[point_count].position.x += path[-1][0]
        wp[point_count].position.y += path[-1][1]
        point_count += 1
    
    group.set_max_velocity_scaling_factor(PAINTING_VELOCITY_SCALING_FACTOR)
    # Requires a modified version of MoveIt that reads these params
    # (MoveIt from gitlab.com/mitroboteam/moveit)
    rospy.set_param("/move_group/cartesian_path_planning/max_velocity_scaling_factor", PAINTING_VELOCITY_SCALING_FACTOR)
    rospy.set_param("/move_group/cartesian_path_planning/enable_eef_constant_speed", True)
    plan, fraction = group.compute_cartesian_path(wp, 0.005, 20)
    return wp, plan, fraction


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_path_demo')

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander('robot')

    # for RViz visualization
    display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20)

    marker_pub = rospy.Publisher(
            '/trajectory_markers',
            visualization_msgs.msg.MarkerArray,
            queue_size=20)

    # Move to desired starting position
    move_to_start(group, 0, GENERAL_VELOCITY_SCALING_FACTOR)
    depth = 0
    while True:
        command = raw_input("Type u/d/confirm to set depth: ")
        if command == "u":
            depth -= 0.005
        elif command == "d":
            depth += 0.005
        elif command == "confirm":
            break
        else:
            print("Invalid input.")
            continue
        move_to_start(group, depth, DEPTH_ADJUST_VELOCITY_SCALING_FACTOR)
        
    # Plan cartesian path
    paths = read_dxf('/root/drawing.dxf')

    plans = []
    
    print("======== Planning cartesian paths")
    wp, plan, fraction = plan_cartesian(group, paths)
    print("Fraction of desired trajectory completed by plan: {}%".format(fraction*100))
    
    # Create and display markers
    add_markers, remove_markers = create_markers(wp)
    marker_pub.publish(add_markers)

    # Execute trajectory
    if fraction > 0.99:
        print("Executing plan")
        group.execute(plan)
    else:
        print("[!] Fraction not high enough. Will not execute plan.")
        time.sleep(5)
        
    marker_pub.publish(remove_markers)

    print("Done")
    moveit_commander.roscpp_shutdown()
