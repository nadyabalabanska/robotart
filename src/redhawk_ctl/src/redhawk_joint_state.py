#!/usr/bin/python

import math
import rospy
from redhawk_ctl.msg import redhawk_telem
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def telem_recv(telem):
    joint_state_msg = JointState()
    joint_state_msg.header = Header()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    joint_state_msg.position = [
        math.pi/180.0*telem.joint_1,
        math.pi/180.0*telem.joint_2,
        -math.pi/180.0*telem.joint_3,
        math.pi/180.0*telem.joint_4,
        -math.pi/180.0*telem.joint_5,
        math.pi/180.0*telem.joint_6
    ]
    pub.publish(joint_state_msg)
   

rospy.init_node('redhawk_joint_state')
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
sub = rospy.Subscriber('redhawk_telem', redhawk_telem, telem_recv)
rospy.spin()
