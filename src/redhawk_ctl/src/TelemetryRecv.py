#!/usr/bin/python 

import socket
import Tkinter
import struct
import thread
import time

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
   
schunkErr = { 0x76 : 'Cable Break Error', 
0xDD : 'Commutation Err', 
0xD2 : 'Config Mem Err',
0xDE : 'Overcurrent',
0xD9 : 'Fast Stop',
0xDC : 'Fragmentation Err',
0xDF : 'I2T Err',
0xE0 : 'Could Not Initialize',
0xE1 : 'Internal Err',
0xD4 : 'Bad Phrase',
0x73 : 'Logic Overvolt',
0x72 : 'Logic Undervolt',
0xEC : 'Math Error',
0x75 : 'Motor Overvolt',
0x74 : 'Motor Undervolt (EStop)',
0x82 : 'Overshoot error',
0xD3 : 'Program Memory Error',
0xEB : 'Resolver Check Fail',
0xD8 : 'Contact Service',
0xD6 : 'Soft High Limit Reached',
0xD5 : 'Soft Low Limit Reached',
0x71 : 'Overtemp',
0x70 : 'Undertemp',
0xE4 : 'Too Fast Error',
0xDA : 'Tow Error',
0xDB : 'Profibus Error',
0xC8 : 'Wrong Ramp Type',
0x01 : '(INFO) Booted Successfully',
0x19 : 'Invalid Checksum',
0x09 : 'Communication Error',
0x05 : 'Command Failed',
0x1D : 'Invalid Message Length',
0x08 : 'No Error',
0x03 : 'Invalid Rights',
0x07 : '(INFO) Sine Vector Search',
0x10 : '(INFO) Comm. Timeout',
0x11 : '(INFO) Unknown Axis',
0x04 : '(INFO) Unknown Command',
0x16 : '(INFO) Incorrect Baudrate',
0x1E : '(INFO) Param Out of Range',
0x06 : 'Not Referenced!',
0x83 : 'Unknown Hardware Version' }
   
throttle = False

class messageSender(Tkinter.Tk):
    def __init__(self,parent):
        Tkinter.Tk.__init__(self,parent)
        self.parent = parent
        self.initialize()
        
    def initialize(self):
        self.grid()
        rowCounter = 0
        
        versionLabel = Tkinter.Label(self,text=u"Version",anchor="w")
        versionLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dVersion = Tkinter.StringVar()
        self.versionEntry = Tkinter.Entry(self,textvariable=self.dVersion)
        self.versionEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        packetNumberLabel = Tkinter.Label(self,text=u"Packet Number",anchor="w")
        packetNumberLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dPacketNumber = Tkinter.StringVar()
        self.packetNumberEntry = Tkinter.Entry(self,textvariable=self.dPacketNumber)
        self.packetNumberEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        idLabel = Tkinter.Label(self,text=u"Robot ID",anchor="w")
        idLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dID = Tkinter.StringVar()
        self.idEntry = Tkinter.Entry(self,textvariable=self.dID)
        self.idEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        xforceLabel = Tkinter.Label(self,text=u"X-Axis Force",anchor="w")
        xforceLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dxForce = Tkinter.StringVar()
        self.xForceEntry = Tkinter.Entry(self,textvariable=self.dxForce)
        self.xForceEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        yforceLabel = Tkinter.Label(self,text=u"Y-Axis Force",anchor="w")
        yforceLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dyForce = Tkinter.StringVar()
        self.yForceEntry = Tkinter.Entry(self,textvariable=self.dyForce)
        self.yForceEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        zforceLabel = Tkinter.Label(self,text=u"Z-Axis Force",anchor="w")
        zforceLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dzForce = Tkinter.StringVar()
        self.zForceEntry = Tkinter.Entry(self,textvariable=self.dzForce)
        self.zForceEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        xtorqueLabel = Tkinter.Label(self,text=u"X-Axis Torque",anchor="w")
        xtorqueLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dxTorque = Tkinter.StringVar()
        self.xTorqueEntry = Tkinter.Entry(self,textvariable=self.dxTorque)
        self.xTorqueEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        ytorqueLabel = Tkinter.Label(self,text=u"Y-Axis Torque",anchor="w")
        ytorqueLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dyTorque = Tkinter.StringVar()
        self.yTorqueEntry = Tkinter.Entry(self,textvariable=self.dyTorque)
        self.yTorqueEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        ztorqueLabel = Tkinter.Label(self,text=u"Z-Axis Torque",anchor="w")
        ztorqueLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dzTorque = Tkinter.StringVar()
        self.zTorqueEntry = Tkinter.Entry(self,textvariable=self.dzTorque)
        self.zTorqueEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        baseForceXLabel = Tkinter.Label(self,text=u"X Force (base frame)",anchor="w")
        baseForceXLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dBaseForceX = Tkinter.StringVar()
        self.baseForceXEntry = Tkinter.Entry(self,textvariable=self.dBaseForceX)
        self.baseForceXEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        baseForceYLabel = Tkinter.Label(self,text=u"Y Force (base frame)",anchor="w")
        baseForceYLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dBaseForceY = Tkinter.StringVar()
        self.baseForceYEntry = Tkinter.Entry(self,textvariable=self.dBaseForceY)
        self.baseForceYEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        baseForceZLabel = Tkinter.Label(self,text=u"Z Force (base frame)",anchor="w")
        baseForceZLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dBaseForceZ = Tkinter.StringVar()
        self.baseForceZEntry = Tkinter.Entry(self,textvariable=self.dBaseForceZ)
        self.baseForceZEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
                
        baseTorqueXLabel = Tkinter.Label(self,text=u"X Torque (base frame)",anchor="w")
        baseTorqueXLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dBaseTorqueX = Tkinter.StringVar()
        self.baseTorqueXEntry = Tkinter.Entry(self,textvariable=self.dBaseTorqueX)
        self.baseTorqueXEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        baseTorqueYLabel = Tkinter.Label(self,text=u"Y Torque (base frame)",anchor="w")
        baseTorqueYLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dBaseTorqueY = Tkinter.StringVar()
        self.baseTorqueYEntry = Tkinter.Entry(self,textvariable=self.dBaseTorqueY)
        self.baseTorqueYEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        baseTorqueZLabel = Tkinter.Label(self,text=u"Z Torque (base frame)",anchor="w")
        baseTorqueZLabel.grid(column=0,row=rowCounter,sticky='EW')
        self.dBaseTorqueZ = Tkinter.StringVar()
        self.baseTorqueZEntry = Tkinter.Entry(self,textvariable=self.dBaseTorqueZ)
        self.baseTorqueZEntry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
                
        joint1Label = Tkinter.Label(self,text=u"Joint 1 Position",anchor="w")
        joint1Label.grid(column=0,row=rowCounter,sticky='EW')
        self.dJoint1 = Tkinter.StringVar()
        self.joint1Entry = Tkinter.Entry(self,textvariable=self.dJoint1)
        self.joint1Entry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        joint2Label = Tkinter.Label(self,text=u"Joint 2 Position",anchor="w")
        joint2Label.grid(column=0,row=rowCounter,sticky='EW')
        self.dJoint2 = Tkinter.StringVar()
        self.joint2Entry = Tkinter.Entry(self,textvariable=self.dJoint2)
        self.joint2Entry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        joint3Label = Tkinter.Label(self,text=u"Joint 3 Position",anchor="w")
        joint3Label.grid(column=0,row=rowCounter,sticky='EW')
        self.dJoint3 = Tkinter.StringVar()
        self.joint3Entry = Tkinter.Entry(self,textvariable=self.dJoint3)
        self.joint3Entry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        joint4Label = Tkinter.Label(self,text=u"Joint 4 Position",anchor="w")
        joint4Label.grid(column=0,row=rowCounter,sticky='EW')
        self.dJoint4 = Tkinter.StringVar()
        self.joint4Entry = Tkinter.Entry(self,textvariable=self.dJoint4)
        self.joint4Entry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        joint5Label = Tkinter.Label(self,text=u"Joint 5 Position",anchor="w")
        joint5Label.grid(column=0,row=rowCounter,sticky='EW')
        self.dJoint5 = Tkinter.StringVar()
        self.joint5Entry = Tkinter.Entry(self,textvariable=self.dJoint5)
        self.joint5Entry.grid(column=1,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        joint6Label = Tkinter.Label(self,text=u"Joint 6 Position",anchor="w")
        joint6Label.grid(column=0,row=rowCounter,sticky='EW')
        self.dJoint6 = Tkinter.StringVar()
        self.joint6Entry = Tkinter.Entry(self,textvariable=self.dJoint6)
        self.joint6Entry.grid(column=1,row=rowCounter,sticky='EW')
        
        #New column!
        rowCounter = 0
        
        basePosXLabel = Tkinter.Label(self,text=u"X Position (base frame)",anchor="w")
        basePosXLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dBasePosX = Tkinter.StringVar()
        self.basePosXEntry = Tkinter.Entry(self,textvariable=self.dBasePosX)
        self.basePosXEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        basePosYLabel = Tkinter.Label(self,text=u"Y Position (base frame)",anchor="w")
        basePosYLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dBasePosY = Tkinter.StringVar()
        self.basePosYEntry = Tkinter.Entry(self,textvariable=self.dBasePosY)
        self.basePosYEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        basePosZLabel = Tkinter.Label(self,text=u"Z Position (base frame)",anchor="w")
        basePosZLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dBasePosZ = Tkinter.StringVar()
        self.basePosZEntry = Tkinter.Entry(self,textvariable=self.dBasePosZ)
        self.basePosZEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
                
        baseRotXLabel = Tkinter.Label(self,text=u"X Rotation (base frame)",anchor="w")
        baseRotXLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dBaseRotX = Tkinter.StringVar()
        self.baseRotXEntry = Tkinter.Entry(self,textvariable=self.dBaseRotX)
        self.baseRotXEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        baseRotYLabel = Tkinter.Label(self,text=u"Y Rotation (base frame)",anchor="w")
        baseRotYLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dBaseRotY = Tkinter.StringVar()
        self.baseRotYEntry = Tkinter.Entry(self,textvariable=self.dBaseRotY)
        self.baseRotYEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        baseRotZLabel = Tkinter.Label(self,text=u"Z Rotation (base frame)",anchor="w")
        baseRotZLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dBaseRotZ = Tkinter.StringVar()
        self.baseRotZEntry = Tkinter.Entry(self,textvariable=self.dBaseRotZ)
        self.baseRotZEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
                
        incXLabel = Tkinter.Label(self,text=u"Inclinometer X",anchor="w")
        incXLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dincX = Tkinter.StringVar()
        self.incXEntry = Tkinter.Entry(self,textvariable=self.dincX)
        self.incXEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        incYLabel = Tkinter.Label(self,text=u"Inclinometer Y",anchor="w")
        incYLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dincY = Tkinter.StringVar()
        self.incYEntry = Tkinter.Entry(self,textvariable=self.dincY)
        self.incYEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        touLabel = Tkinter.Label(self,text=u"Time of Use",anchor="w")
        touLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dTou = Tkinter.StringVar()
        self.touEntry = Tkinter.Entry(self,textvariable=self.dTou)
        self.touEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        buttonLabel = Tkinter.Label(self,text=u"Box Button Status",anchor="w")
        buttonLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dButton = Tkinter.StringVar()
        self.buttonEntry = Tkinter.Entry(self,textvariable=self.dButton)
        self.buttonEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        battLabel = Tkinter.Label(self,text=u"Battery Voltage",anchor="w")
        battLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dBatt = Tkinter.StringVar()
        self.battEntry = Tkinter.Entry(self,textvariable=self.dBatt)
        self.battEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        validationLabel = Tkinter.Label(self,text=u"Validation Packet #",anchor="w")
        validationLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dValidation = Tkinter.StringVar()
        self.validationEntry = Tkinter.Entry(self,textvariable=self.dValidation)
        self.validationEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        sysErrLabel = Tkinter.Label(self,text=u"System error code",anchor="w")
        sysErrLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dSysErr = Tkinter.StringVar()
        self.sysErrEntry = Tkinter.Entry(self,textvariable=self.dSysErr)
        self.sysErrEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        JointErr1Label = Tkinter.Label(self,text=u"Joint 1 Error",anchor="w")
        JointErr1Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dJointErr1 = Tkinter.StringVar()
        self.JointErr1Entry = Tkinter.Entry(self,textvariable=self.dJointErr1)
        self.JointErr1Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        JointErr2Label = Tkinter.Label(self,text=u"Joint 2 Error",anchor="w")
        JointErr2Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dJointErr2 = Tkinter.StringVar()
        self.JointErr2Entry = Tkinter.Entry(self,textvariable=self.dJointErr2)
        self.JointErr2Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        JointErr3Label = Tkinter.Label(self,text=u"Joint 3 Error",anchor="w")
        JointErr3Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dJointErr3 = Tkinter.StringVar()
        self.JointErr3Entry = Tkinter.Entry(self,textvariable=self.dJointErr3)
        self.JointErr3Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        JointErr4Label = Tkinter.Label(self,text=u"Joint 4 Error",anchor="w")
        JointErr4Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dJointErr4 = Tkinter.StringVar()
        self.JointErr4Entry = Tkinter.Entry(self,textvariable=self.dJointErr4)
        self.JointErr4Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        JointErr5Label = Tkinter.Label(self,text=u"Joint 5 Error",anchor="w")
        JointErr5Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dJointErr5 = Tkinter.StringVar()
        self.JointErr5Entry = Tkinter.Entry(self,textvariable=self.dJointErr5)
        self.JointErr5Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        JointErr6Label = Tkinter.Label(self,text=u"Joint 6 Error",anchor="w")
        JointErr6Label.grid(column=2,row=rowCounter,sticky='EW')
        self.dJointErr6 = Tkinter.StringVar()
        self.JointErr6Entry = Tkinter.Entry(self,textvariable=self.dJointErr6)
        self.JointErr6Entry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        cmdCompleteLabel = Tkinter.Label(self,text=u"Position Command Complete",anchor="w")
        cmdCompleteLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dCmdComplete = Tkinter.StringVar()
        self.cmdCopmleteEntry = Tkinter.Entry(self,textvariable=self.dCmdComplete)
        self.cmdCopmleteEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
        crcLabel = Tkinter.Label(self,text=u"CRC",anchor="w")
        crcLabel.grid(column=2,row=rowCounter,sticky='EW')
        self.dCrc = Tkinter.StringVar()
        self.crcEntry = Tkinter.Entry(self,textvariable=self.dCrc)
        self.crcEntry.grid(column=3,row=rowCounter,sticky='EW')
        rowCounter = rowCounter+1
        
   
def runReceiver(app,nothing):
    global throttle
    rcvSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rcvSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rcvSock.bind(("", 42121))

    while True:
       data, addr = rcvSock.recvfrom(1024)
       telemetryFormat = '!f 2I 27f I f 9I H'
       telemetry = struct.unpack(telemetryFormat, data)
       
       if (throttle == True):
           time.sleep(0.01)
           continue
       throttle = True
       
       app.dVersion.set(telemetry[0])
       app.dPacketNumber.set(telemetry[1])
       app.dID.set(telemetry[2])
       app.dxForce.set(telemetry[3])
       app.dyForce.set(telemetry[4])
       app.dzForce.set(telemetry[5])
       app.dxTorque.set(telemetry[6])
       app.dyTorque.set(telemetry[7])
       app.dzTorque.set(telemetry[8])
       app.dBaseForceX.set(telemetry[9])
       app.dBaseForceY.set(telemetry[10])
       app.dBaseForceZ.set(telemetry[11])
       app.dBaseTorqueX.set(telemetry[12])
       app.dBaseTorqueY.set(telemetry[13])
       app.dBaseTorqueZ.set(telemetry[14])
       app.dJoint1.set(telemetry[15])
       app.dJoint2.set(telemetry[16])
       app.dJoint3.set(telemetry[17])
       app.dJoint4.set(telemetry[18])
       app.dJoint5.set(telemetry[19])
       app.dJoint6.set(telemetry[20])
       app.dBasePosX.set(telemetry[21])
       app.dBasePosY.set(telemetry[22])
       app.dBasePosZ.set(telemetry[23])
       app.dBaseRotX.set(telemetry[24])
       app.dBaseRotY.set(telemetry[25])
       app.dBaseRotZ.set(telemetry[26])
       app.dincX.set(telemetry[27])
       app.dincY.set(telemetry[28])
       app.dTou.set(telemetry[29])
       app.dButton.set(telemetry[30])
       app.dBatt.set(telemetry[31])
       app.dValidation.set(telemetry[32])
       app.dSysErr.set(telemetry[33])
       app.dJointErr1.set(getJointStatus(telemetry[34]))
       if(telemetry[34] == 0x08):
           app.JointErr1Entry.config(bg='green')
       else:
           app.JointErr1Entry.config(bg='red')
       app.dJointErr2.set(getJointStatus(telemetry[35]))
       if(telemetry[35] == 0x08):
           app.JointErr2Entry.config(bg='green')
       else:
           app.JointErr2Entry.config(bg='red')
       app.dJointErr3.set(getJointStatus(telemetry[36]))
       if(telemetry[36] == 0x08):
           app.JointErr3Entry.config(bg='green')
       else:
           app.JointErr3Entry.config(bg='red')
       app.dJointErr4.set(getJointStatus(telemetry[37]))
       if(telemetry[37] == 0x08):
           app.JointErr4Entry.config(bg='green')
       else:
           app.JointErr4Entry.config(bg='red')
       app.dJointErr5.set(getJointStatus(telemetry[38]))
       if(telemetry[38] == 0x08):
           app.JointErr5Entry.config(bg='green')
       else:
           app.JointErr5Entry.config(bg='red')
       app.dJointErr6.set(getJointStatus(telemetry[39]))
       if(telemetry[39] == 0x08):
           app.JointErr6Entry.config(bg='green')
       else:
           app.JointErr6Entry.config(bg='red')
       app.dCmdComplete.set(telemetry[40])
       app.dCrc.set('0x%04X' % telemetry[41])
       theCrc = crc(data[0:-2])
       if (theCrc == telemetry[41]):
           app.crcEntry.config(bg='green')
       else:
           app.crcEntry.config(bg='red')
           print "Got: 0x%04X Expected: 0x%04X" % (telemetry[41], theCrc)
           
def getJointStatus(errValue):
    errName = '0x%02X' % errValue
    try:
        if(schunkErr[errValue]):
            errName = schunkErr[errValue] + ' (' + errName +')'
    except:
        pass
    return errName
           
def throttle(app,nothing):
    global throttle
    while True:
        throttle = False;
        time.sleep(0.1)

def crc(data):
    uCRCValue = 0
    for char in data:
        c = ord(char)
        uCRCValue = CRC16IBMTable[(uCRCValue ^ c) & 0xFF] ^ (uCRCValue >> 8);
    return uCRCValue;

if __name__ == "__main__":
    app = messageSender(None)
    app.title('Manipulator Telemetry')
    thread.start_new_thread(runReceiver,(app,""))
    thread.start_new_thread(throttle,(app,""))
    app.mainloop()
    
