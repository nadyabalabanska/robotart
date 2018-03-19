#!/usr/bin/env python2

import thread
import time

import rospy
from redhawk_ctl.msg import redhawk_cmd
from redhawk_ctl.msg import redhawk_telem
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


PUB_RATE = 60
CMD_TYPE_VELOCITY = 0
CMD_TYPE_POSITION = 1
VEL_MULT = 1

class RedhawkSim():
    def __init__(self):
        self.cmd_type = CMD_TYPE_VELOCITY
        self.joint = [0, 0, 0, 0, 0, 0]
        self.angles = [0, 0, 0, 0, 0, 0]
        self.limits = [(-200, 200),
                       (-90, 90),
                       (-180, 180),
                       None,
                       (-180, 180),
                       None]
        self.sign = [1, 1, 1, 1, 1, 1]
        self.max_accel = 0
        self.max_vel = 0
        self.prev_time = 0

    def update(self):
        now = rospy.Time.now().to_sec()
        if self.prev_time != 0:
            delta = now - self.prev_time
            for i in range(len(self.angles)):
                if self.cmd_type == CMD_TYPE_POSITION:
                    self.angles[i] = self.joint[i]
                elif self.cmd_type == CMD_TYPE_VELOCITY:
                    self.angles[i] += self.joint[i] * delta * self.sign[i] * VEL_MULT
                    
                # Clip range
                if self.limits[i] != None:
                    self.angles[i] = min(self.limits[i][1],
                                         max(self.limits[i][0],
                                             self.angles[i]))
                    
        self.prev_time = now

    def stopCommand(self):
        self.joint = [0, 0, 0, 0, 0, 0]
    
    def process_cmd(self, msg):
        if msg.arm_enable:
            # Send move command

            # Set command type
            if msg.cmd_type == msg.CMD_POSITION:
                if self.cmd_type != CMD_TYPE_POSITION:
                    self.stopCommand()
                self.cmd_type = CMD_TYPE_POSITION
            elif msg.cmd_type == msg.CMD_VELOCITY:
                if self.cmd_type != CMD_TYPE_VELOCITY:
                    self.stopCommand()
                self.cmd_type = CMD_TYPE_VELOCITY
            else:
                self.stopCommand()
                return

            if msg.max_accel != self.max_accel or msg.max_vel != self.max_vel:
                self.max_accel = msg.max_accel
                self.max_vel = msg.max_vel
                self.stopCommand()
                            
            self.joint[0] = msg.joint_1
            self.joint[1] = msg.joint_2
            self.joint[2] = msg.joint_3
            self.joint[3] = msg.joint_4
            self.joint[4] = msg.joint_5
            self.joint[5] = msg.joint_6
        else:
            self.stopCommand()

    def pub_telem(self, pub):
        msg = redhawk_telem()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.joint_1 = self.angles[0]
        msg.joint_2 = self.angles[1]
        msg.joint_3 = self.angles[2]
        msg.joint_4 = self.angles[3]
        msg.joint_5 = self.angles[4]
        msg.joint_6 = self.angles[5]
        pub.publish(msg)
     
    
if __name__ == "__main__":
    rospy.init_node('redhawk_sim')

    sim = RedhawkSim()

    rospy.Subscriber('redhawk_cmd', redhawk_cmd, sim.process_cmd)
    pub = rospy.Publisher('redhawk_telem', redhawk_telem, queue_size = 1000)

    rate = rospy.Rate(PUB_RATE)

    while not rospy.is_shutdown():
        sim.update()
        sim.pub_telem(pub)
        rate.sleep()
