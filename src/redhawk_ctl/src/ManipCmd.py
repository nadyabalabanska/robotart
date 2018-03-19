#!/usr/bin/python 

import Tkinter
import socket
import struct
import thread
import time

UDP_IP = "172.21.0.31"
UDP_PORT = 42120

data = [1.0, 0, 172, 20, 0, 14, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0xD48D]
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
   
watchdogEnable = 0

class messageSender(Tkinter.Tk):
    def __init__(self,parent):
        Tkinter.Tk.__init__(self,parent)
        self.parent = parent
        self.initialize()
        
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
                
    def initialize(self):
        self.grid()
        vcmd = (self.register(self.validate), '%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        vcmdint = (self.register(self.validateint), '%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        
        rowCounter = 0
        
        versionLabel = Tkinter.Label(self,text=u"Message Version:")
        versionLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dVersion = Tkinter.StringVar()
        self.dVersion.set('1.0')
        self.versionEntry = Tkinter.Entry(self,textvariable=self.dVersion,validate='key',validatecommand=vcmd)
        self.versionEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        packetCountLabel = Tkinter.Label(self,text=u"Packet Count:")
        packetCountLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dPacketCount = Tkinter.StringVar()
        self.dPacketCount.set('0')
        self.entry = Tkinter.Entry(self,textvariable=self.dPacketCount,validate='key',validatecommand=vcmdint)
        self.entry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        IP1Label = Tkinter.Label(self,text=u"IP Address Octet 1",anchor="w")
        IP1Label.grid(column=0,row=rowCounter,sticky='EW')
        self.dIP1 = Tkinter.StringVar()
        self.dIP1.set('172')
        self.dIP1Entry = Tkinter.Entry(self,textvariable=self.dIP1,validate='key',validatecommand=vcmdint)
        self.dIP1Entry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        IP2Label = Tkinter.Label(self,text=u"IP Address Octet 2",anchor="w")
        IP2Label.grid(column=0,row=rowCounter,sticky='EW')
        self.dIP2 = Tkinter.StringVar()
        self.dIP2.set('20')
        self.dIP2Entry = Tkinter.Entry(self,textvariable=self.dIP2,validate='key',validatecommand=vcmdint)
        self.dIP2Entry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        IP3Label = Tkinter.Label(self,text=u"IP Address Octet 3",anchor="w")
        IP3Label.grid(column=0,row=rowCounter,sticky='EW')
        self.dIP3 = Tkinter.StringVar()
        self.dIP3.set('0')
        self.dIP3Entry = Tkinter.Entry(self,textvariable=self.dIP3,validate='key',validatecommand=vcmdint)
        self.dIP3Entry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        IP4Label = Tkinter.Label(self,text=u"IP Address Octet 4",anchor="w")
        IP4Label.grid(column=0,row=rowCounter,sticky='EW')
        self.dIP4 = Tkinter.StringVar()
        self.dIP4.set('14')
        self.dIP4Entry = Tkinter.Entry(self,textvariable=self.dIP4,validate='key',validatecommand=vcmdint)
        self.dIP4Entry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        armEnableLabel = Tkinter.Label(self,text=u"Arm Enable")
        armEnableLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dArmEnable = Tkinter.StringVar()
        self.dArmEnable.set('1')
        self.armEnableEntry = Tkinter.Entry(self,textvariable=self.dArmEnable,validate='key',validatecommand=vcmdint)
        self.armEnableEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        gripTareLabel = Tkinter.Label(self,text=u"Gripper Tare")
        gripTareLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dGripTare = Tkinter.StringVar()
        self.dGripTare.set('0')
        self.gripTareEntry = Tkinter.Entry(self,textvariable=self.dGripTare,validate='key',validatecommand=vcmdint)
        self.gripTareEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        wristTareLabel = Tkinter.Label(self,text=u"Wrist Tare")
        wristTareLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dWristTare = Tkinter.StringVar()
        self.dWristTare.set('0')
        self.wristTareEntry = Tkinter.Entry(self,textvariable=self.dWristTare,validate='key',validatecommand=vcmdint)
        self.wristTareEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        cmdTypeLabel = Tkinter.Label(self,text=u"Command Type")
        cmdTypeLabel.grid(column=0,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1

        cmdTypeLabel = Tkinter.Label(self,text=u"     0 = Velocity")
        cmdTypeLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dCmdType = Tkinter.StringVar()
        self.dCmdType.set('0')
        self.cmdTypeEntry = Tkinter.Entry(self,textvariable=self.dCmdType,validate='key',validatecommand=vcmdint)
        self.cmdTypeEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        cmdTypeLabel = Tkinter.Label(self,text=u"     1 = Position")
        cmdTypeLabel.grid(column=0,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1

        toolXLabel = Tkinter.Label(self,text=u"Tool X Offset")
        toolXLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dToolX = Tkinter.StringVar()
        self.dToolX.set('0')
        self.toolXEntry = Tkinter.Entry(self,textvariable=self.dToolX,validate='key',validatecommand=vcmd)
        self.toolXEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        toolYLabel = Tkinter.Label(self,text=u"Tool Y Offset")
        toolYLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dToolY = Tkinter.StringVar()
        self.dToolY.set('0')
        self.toolYEntry = Tkinter.Entry(self,textvariable=self.dToolY,validate='key',validatecommand=vcmd)
        self.toolYEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        toolZLabel = Tkinter.Label(self,text=u"Tool Z Offset")
        toolZLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dToolZ = Tkinter.StringVar()
        self.dToolZ.set('0')
        self.toolZEntry = Tkinter.Entry(self,textvariable=self.dToolZ,validate='key',validatecommand=vcmd)
        self.toolZEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        maxrows = rowCounter
        
        #New column
        rowCounter = 0
        
        joint1Label = Tkinter.Label(self,text=u"Joint 1 Command",anchor="w")
        joint1Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dJoint1 = Tkinter.StringVar()
        self.dJoint1.set('0.0')
        self.joint1Entry = Tkinter.Entry(self,textvariable=self.dJoint1,validate='key',validatecommand=vcmd)
        self.joint1Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        joint2Label = Tkinter.Label(self,text=u"Joint 2 Command",anchor="w")
        joint2Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dJoint2 = Tkinter.StringVar()
        self.dJoint2.set('0.0')
        self.joint2Entry = Tkinter.Entry(self,textvariable=self.dJoint2,validate='key',validatecommand=vcmd)
        self.joint2Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        joint3Label = Tkinter.Label(self,text=u"Joint 3 Command",anchor="w")
        joint3Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dJoint3 = Tkinter.StringVar()
        self.dJoint3.set('0.0')
        self.joint3Entry = Tkinter.Entry(self,textvariable=self.dJoint3,validate='key',validatecommand=vcmd)
        self.joint3Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        joint4Label = Tkinter.Label(self,text=u"Joint 4 Command",anchor="w")
        joint4Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dJoint4 = Tkinter.StringVar()
        self.dJoint4.set('0.0')
        self.joint4Entry = Tkinter.Entry(self,textvariable=self.dJoint4,validate='key',validatecommand=vcmd)
        self.joint4Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        joint5Label = Tkinter.Label(self,text=u"Joint 5 Command",anchor="w")
        joint5Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dJoint5 = Tkinter.StringVar()
        self.dJoint5.set('0.0')
        self.joint5Entry = Tkinter.Entry(self,textvariable=self.dJoint5,validate='key',validatecommand=vcmd)
        self.joint5Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        joint6Label = Tkinter.Label(self,text=u"Joint 6 Command",anchor="w")
        joint6Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dJoint6 = Tkinter.StringVar()
        self.dJoint6.set('0.0')
        self.joint6Entry = Tkinter.Entry(self,textvariable=self.dJoint6,validate='key',validatecommand=vcmd)
        self.joint6Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        timingLabel = Tkinter.Label(self,text=u"Max Acceleration",anchor="w")
        timingLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dMaxAcceleration = Tkinter.StringVar()
        self.dMaxAcceleration.set('45.0')
        self.timingEntry = Tkinter.Entry(self,textvariable=self.dMaxAcceleration,validate='key',validatecommand=vcmd)
        self.timingEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        gripperEffortLabel = Tkinter.Label(self,text=u"Max Velocity",anchor="w")
        gripperEffortLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dMaxVelocity = Tkinter.StringVar()
        self.dMaxVelocity.set('30.0')
        self.gripperEffortEntry = Tkinter.Entry(self,textvariable=self.dMaxVelocity,validate='key',validatecommand=vcmd)
        self.gripperEffortEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        lightsLabel = Tkinter.Label(self,text=u"Light Control",anchor="w")
        lightsLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dLights = Tkinter.StringVar()
        self.dLights.set('0')
        self.lightsEntry = Tkinter.Entry(self,textvariable=self.dLights,validate='key',validatecommand=vcmd)
        self.lightsEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        camera1Label = Tkinter.Label(self,text=u"Camera Output 1",anchor="w")
        camera1Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dCamera1 = Tkinter.StringVar()
        self.dCamera1.set('0')
        self.camera1Entry = Tkinter.Entry(self,textvariable=self.dCamera1,validate='key',validatecommand=vcmdint)
        self.camera1Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        camera2Label = Tkinter.Label(self,text=u"Camera Output 2",anchor="w")
        camera2Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dCamera2 = Tkinter.StringVar()
        self.dCamera2.set('0')
        self.camera2Entry = Tkinter.Entry(self,textvariable=self.dCamera2,validate='key',validatecommand=vcmdint)
        self.camera2Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        camera3Label = Tkinter.Label(self,text=u"Camera Output 3",anchor="w")
        camera3Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dCamera3 = Tkinter.StringVar()
        self.dCamera3.set('0')
        self.camera3Entry = Tkinter.Entry(self,textvariable=self.dCamera3,validate='key',validatecommand=vcmdint)
        self.camera3Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        camera4Label = Tkinter.Label(self,text=u"Camera Output 4",anchor="w")
        camera4Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dCamera4 = Tkinter.StringVar()
        self.dCamera4.set('0')
        self.camera4Entry = Tkinter.Entry(self,textvariable=self.dCamera4,validate='key',validatecommand=vcmdint)
        self.camera4Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        HVEnableLabel = Tkinter.Label(self,text=u"High Voltage Enable",anchor="w")
        HVEnableLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dHVEnable = Tkinter.StringVar()
        self.dHVEnable.set('0')
        self.HVEnableEntry = Tkinter.Entry(self,textvariable=self.dHVEnable,validate='key',validatecommand=vcmdint)
        self.HVEnableEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        LogicEnableLabel = Tkinter.Label(self,text=u"Logic Voltage Enable",anchor="w")
        LogicEnableLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dLogicEnable = Tkinter.StringVar()
        self.dLogicEnable.set('0')
        self.LogicEnableEntry = Tkinter.Entry(self,textvariable=self.dLogicEnable,validate='key',validatecommand=vcmdint)
        self.LogicEnableEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        if (rowCounter > maxrows):  
            maxrows = rowCounter
        
        CRCLabel = Tkinter.Label(self,text=u"Calculated CRC: ",anchor="w")
        CRCLabel.grid(column=0,columnspan=2,row=maxrows,sticky='E')
        self.dCRC = Tkinter.StringVar()
        self.dCRC.set('0')
        self.CRCEntry = Tkinter.Entry(self,textvariable=self.dCRC,validate='key',validatecommand=vcmdint)
        self.CRCEntry.grid(column=2,columnspan=2,row=maxrows,sticky='W')
        maxrows = maxrows+1
        
        sendButton = Tkinter.Button(self,text=u"Send Command",height=5,command=self.moveCommand)
        sendButton.grid(column=0,columnspan=2,row=maxrows,sticky='EW')
        
        stopButton = Tkinter.Button(self,text=u"Send Stop Command",height=5,bg="red",activebackground="red",command=self.stopCommand)
        stopButton.grid(column=2,columnspan=2,row=maxrows,sticky='EW')
        maxrows = maxrows+1
        
        self.watchdogButton = Tkinter.Button(self,text=u"Start Watchdog",height=5,command=self.toggleWatchdog)
        self.watchdogButton.grid(column=0,columnspan=4,row=maxrows)
        
        
    def moveCommand(self):
        global data
        try:
            self.dPacketCount.set(int(self.dPacketCount.get()) + 1)
            data[0] = float(self.dVersion.get())
            data[1] = int(self.dPacketCount.get())
            data[2] = int(self.dIP1.get())
            data[3] = int(self.dIP2.get())
            data[4] = int(self.dIP3.get())
            data[5] = int(self.dIP4.get())
            data[6] = int(self.dArmEnable.get())
            data[7] = int(self.dGripTare.get())
            data[8] = int(self.dWristTare.get())
            data[9] = int(self.dCmdType.get())
            data[10] = float(self.dToolX.get())
            data[11] = float(self.dToolY.get())
            data[12] = float(self.dToolZ.get())
            data[13] = float(self.dJoint1.get())
            data[14] = float(self.dJoint2.get())
            data[15] = float(self.dJoint3.get())
            data[16] = float(self.dJoint4.get())
            data[17] = float(self.dJoint5.get())
            data[18] = float(self.dJoint6.get())
            data[19] = float(self.dMaxAcceleration.get())
            data[20] = float(self.dMaxVelocity.get())
            data[21] = int(self.dLights.get())
            data[22] = int(self.dCamera1.get())
            data[23] = int(self.dCamera2.get())
            data[24] = int(self.dCamera3.get())
            data[25] = int(self.dCamera4.get())
            data[26] = int(self.dHVEnable.get())
            data[27] = int(self.dLogicEnable.get())
        except ValueError:
            print 'Value invalid! Fix number formatting and try again.'
            return
            
        self.sendMessage(data)
        
    def stopCommand(self):
        global data
        try:
            self.dPacketCount.set(int(self.dPacketCount.get()) + 1)
            data[0] = float(self.dVersion.get())
            data[1] = int(self.dPacketCount.get())
            data[2] = int(self.dIP1.get())
            data[3] = int(self.dIP2.get())
            data[4] = int(self.dIP3.get())
            data[5] = int(self.dIP4.get())
            data[6] = 0 # Arm disable
            data[7] = 0 # Gripper Tare
            data[8] = 0 # Wrist Tare
            data[9] = 0 # Velocity Mode
            data[10] = float(self.dToolX.get())
            data[11] = float(self.dToolY.get())
            data[12] = float(self.dToolZ.get())
            data[13] = 0.0 # Joint 1
            data[14] = 0.0 # Joint 2
            data[15] = 0.0 # Joint 3
            data[16] = 0.0 # Joint 4
            data[17] = 0.0 # Joint 5
            data[18] = 0.0 # Joint 6
            data[19] = float(self.dMaxAcceleration.get())
            data[20] = float(self.dMaxVelocity.get())
            data[21] = int(self.dLights.get())
            data[22] = int(self.dCamera1.get())
            data[23] = int(self.dCamera2.get())
            data[24] = int(self.dCamera3.get())
            data[25] = int(self.dCamera4.get())
            data[26] = int(self.dHVEnable.get())
            data[27] = int(self.dLogicEnable.get())
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
        self.dCRC.set("0x%04X" % self.crc(packet))
        
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
    app = messageSender(None)
    app.title('Manipulator Test Tool')
    thread.start_new_thread(runWatchdog,())
    app.mainloop()
    

