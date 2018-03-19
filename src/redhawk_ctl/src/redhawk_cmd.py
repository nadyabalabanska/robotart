#!/usr/bin/python 

import socket
import struct
import thread
import time

import rospy
from redhawk_ctl.msg import redhawk_cmd

PUB_RATE = 60

CONTROLLER_IP = [172, 21, 0, 123]
UDP_IP = "172.21.0.31"
UDP_PORT = 42120

data = [1.0, 0] + CONTROLLER_IP + [1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0xD48D]
CRC16IBMTable = [0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
   0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
   0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
   0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
   0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
   0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
   0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
   0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
   0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
   0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
   0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
   0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
   0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
   0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
   0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
   0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
   0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
   0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
   0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
   0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
   0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
   0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
   0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
   0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
   0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
   0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
   0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
   0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
   0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
   0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
   0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
   0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040]
   

CMD_TYPE_VELOCITY = 0
CMD_TYPE_POSITION = 1

class messageSender():
    def __init__(self):
        self.packet_count = 0
        self.version = 1.0
        self.telemetry_ip = CONTROLLER_IP
        self.tool_x = 0
        self.tool_y = 0
        self.tool_z = 0
        self.camera = [0, 0, 0, 0]
        self.arm_enable = 1
        self.grip_tare = 0
        self.wrist_tare = 1
        self.cmd_type = CMD_TYPE_POSITION
        self.joint = [0, 0, 0, 0, 0, 0]
        self.max_accel = 20
        self.max_vel = 30
        self.lights = 0
        self.high_voltage_en = 0
        self.logic_voltage_en = 0

        
    def validate(self, action, index, value_if_allowed, prior_value, text, validate_type, trigger_type, widget_name):
        if text in '0123456789.+-':
            if (value_if_allowed.count('.') < 2):
                return True
            else:
                return False
        else:
            return False
                
    def validateint(self, action, index, value_if_allowed, prior_value, text, validate_type, trigger_type, widget_name):
        if text in '0123456789':
            return True
        else:
            return False
        
        
    def moveCommand(self):
        self.packet_count += 1
        data[0] = self.version
        data[1] = self.packet_count
        data[2] = self.telemetry_ip[0]
        data[3] = self.telemetry_ip[1]
        data[4] = self.telemetry_ip[2]
        data[5] = self.telemetry_ip[3]
        data[6] = self.arm_enable = 1
        data[7] = self.grip_tare
        data[8] = self.wrist_tare
        data[9] = self.cmd_type
        data[10] = self.tool_x
        data[11] = self.tool_y
        data[12] = self.tool_z
        data[13] = self.joint[0]
        data[14] = self.joint[1]
        data[15] = self.joint[2]
        data[16] = self.joint[3]
        data[17] = self.joint[4]
        data[18] = self.joint[5]
        data[19] = self.max_accel
        data[20] = self.max_vel
        data[21] = self.lights
        data[22] = self.camera[0]
        data[23] = self.camera[1]
        data[24] = self.camera[2]
        data[25] = self.camera[3]
        data[26] = self.high_voltage_en
        data[27] = self.logic_voltage_en
        self.sendMessage(data)
        
    def stopCommand(self):
        self.arm_enable = 0
        self.gripper_tare = 0
        self.wrist_tare = 0
        self.cmd_type = CMD_TYPE_VELOCITY
        try:
            data[6:9] = [0, 0, 0] # Arm disable
            data[7] = 0 # Gripper Tare
            data[8] = 0 # Wrist Tare
            data[9] = 0 # Velocity Mode
            data[10] = self.tool_x
            data[11] = self.tool_y
            data[12] = self.tool_z
            data[13] = 0.0 # Joint 1
            data[14] = 0.0 # Joint 2
            data[15] = 0.0 # Joint 3
            data[16] = 0.0 # Joint 4
            data[17] = 0.0 # Joint 5
            data[18] = 0.0 # Joint 6
            data[19] = self.max_accel
            data[20] = self.max_vel
            data[21] = self.lights
            data[22:26] = self.camera
            data[26] = self.high_voltage_en
            data[27] = self.logic_voltage_en
        except ValueError:
            # In emergency, make sure to send SOMETHING that stops the 
            # arm even if the inputs are invalid
            data[0] = 1.0
            data[1] = 10000000
            data[2] = 172
            data[3] = 21
            data[4] = 0
            data[5] = 31
            data[6] = 0 # Arm disable
            data[7] = 0 # Gripper Tare
            data[8] = 0 # Wrist Tare
            data[9] = 0 # Velocity Mode
            data[10] = 0.0
            data[11] = 0.0
            data[12] = 0.0
            data[13] = 0.0 # Joint 1
            data[14] = 0.0 # Joint 2
            data[15] = 0.0 # Joint 3
            data[16] = 0.0 # Joint 4
            data[17] = 0.0 # Joint 5
            data[18] = 0.0 # Joint 6
            data[19] = 45.0
            data[20] = 30.0
            data[21] = 0
            data[22] = 0
            data[23] = 0
            data[24] = 0
            data[25] = 0
            data[26] = 0
            data[27] = 0
            print 'Value invalid! Fix number formatting and try again.'
            
        self.sendMessage(data)
        
    def sendMessage(self, packetData):
        packet = struct.pack("!f 9I 11f 7I", *packetData[0:-1])
        packetData[28] = self.crc(packet)
        self.dCRC = "0x%04X" % self.crc(packet)
        
        packet = struct.pack("!f 9I 11f 7I H", *packetData)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(packet, (UDP_IP, UDP_PORT))        
        
    def crc(self, data):
        uCRCValue = 0
        for char in data:
            c = ord(char)
            uCRCValue = CRC16IBMTable[(uCRCValue ^ c) & 0xFF] ^ (uCRCValue >> 8);
        return uCRCValue;
        
        
    def toggleWatchdog(self):
        global watchdogEnable
        if(watchdogEnable == 0):
            watchdogEnable = 1
            self.watchdogButton.config(text=u"Stop Watchdog")
        else:
            watchdogEnable = 0
            self.watchdogButton.config(text=u"Start Watchdog")

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

            self.moveCommand()
        else:
            self.stopCommand()
            
        
def runWatchdog():
    WATCHDOG_IP = "172.21.0.31"
    WATCHDOG_PORT = 42122
    watchdogdata = 0xA5A5A5A5
    watchdogpacket = struct.pack("!I", watchdogdata)
    watchdogsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    global watchdogEnable
    while True:
        if(watchdogEnable == 1):
            watchdogsock.sendto(watchdogpacket, (WATCHDOG_IP, WATCHDOG_PORT))
        time.sleep(0.02) # 50Hz

        
if __name__ == "__main__":
    rospy.init_node('redhawk_ctl')

    msg = messageSender()

    rospy.Subscriber('redhawk_cmd', redhawk_cmd, msg.process_cmd)

    rate = rospy.Rate(PUB_RATE)

    global watchdogEnable
    watchdogEnable = True
    thread.start_new_thread(runWatchdog,())

    while not rospy.is_shutdown():
        rate.sleep()
        
    
    

