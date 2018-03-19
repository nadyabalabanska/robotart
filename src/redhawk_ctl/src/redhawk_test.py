#!/usr/bin/env python2

import rospy
from redhawk_ctl.msg import redhawk_cmd

if __name__ == '__main__':
    rospy.init_node('redhawk_test')
    pub = rospy.Publisher('redhawk_cmd', redhawk_cmd)

    r = rospy.Rate(30)

    cmd = redhawk_cmd()

    try:
        while not rospy.is_shutdown():
            cmd.cmd_type = cmd.CMD_POSITION
            cmd.max_vel = 30
            cmd.max_accel = 45
            cmd.joint_1 = 0
            cmd.joint_2 = 0
            cmd.joint_3 = 0
            cmd.joint_4 = 0
            cmd.joint_5 = 0
            cmd.joint_6 = 0
            cmd.arm_enable = True

            pub.publish(cmd)
            r.sleep()
    except KeyboardInterrupt:
        print 'keyboard interrupt'
        cmd.cmd_type = -1
        pub.publish(cmd)
