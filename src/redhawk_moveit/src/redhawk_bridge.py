#!/usr/bin/env python2

from __future__ import print_function
import sys
import math

import time
import rospy
import threading
import redhawk_commander

import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg


class RedhawkActionServer:
    _feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryResult()

    def __init__(self, name, ctl):
        self._action_name = name + '/follow_joint_trajectory'
        self._ctl = ctl
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        self.got_trajectory(goal)

    def got_trajectory(self, goal):
        rospy.loginfo("Got trajectory. Executing...")
        #self._ctl.telem_purge(1) # Get telemetry going
        #self._feedback.positions = self._ctl._telem()
        success = self._ctl.run2(goal.trajectory)
        #self._feedback.positions = self._ctl._telem()
        if success:
            self._as.set_succeeded(self._result)
        else:
            self._as.set_aborted(self._result)
        rospy.loginfo("Trajectory executed.")


if __name__ == '__main__':
    rospy.init_node('redhawk_controller')

    ctl = redhawk_commander.RedhawkController()

    #sub = rospy.Subscriber('/redhawk/redhawk_controller/follow_joint_trajectory/goal', control_msgs.msg.FollowJointTrajectoryActionGoal, got_trajectory, ctl)

    #r = rospy.Rate(60)
    rospy.loginfo("Redhawk trajectory action server for action '{}'".format(rospy.get_name()))
    server = RedhawkActionServer(rospy.get_name(), ctl)
    rospy.spin()
