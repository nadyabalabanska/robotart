#!/usr/bin/env python2

from __future__ import print_function
import sys
import math
import rospy
from time import time, sleep
from redhawk_ctl.msg import redhawk_cmd
from sensor_msgs.msg import Joy
import redhawk_commander


jointmap = [6, 10, 11, 8, 9, 7]
JOINT1 = 7
JOINT6 = 8


def callback(joy, ctl):
    ctl.set_velocities([
        -joy.axes[2] * 20 * joy.buttons[jointmap[0]],
        joy.axes[1] * 20 * joy.buttons[jointmap[1]],
        -joy.axes[1] * 20 * joy.buttons[jointmap[2]],
        -joy.axes[0] * 20 * joy.buttons[jointmap[3]],
        -joy.axes[1] * 20 * joy.buttons[jointmap[4]],
        -joy.axes[2] * 20 * joy.buttons[jointmap[5]]])


if __name__ == '__main__':
    rospy.init_node('redhawk_joy')

    r = rospy.Rate(60)
    
    ctl = redhawk_commander.RedhawkController()
    ctl.stop()
    #ctl.set_pose([0, 0, 0, 0, 0, 0])

    rospy.Subscriber("joy", Joy, callback, ctl)

    rospy.spin()
