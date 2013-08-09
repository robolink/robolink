#!/usr/bin/env python  
import roslib

roslib.load_manifest('robolink')
import rospy

from EposManager.msg import GroupMotorInfo
from EposManager.msg import EPOSControl
from EposManager.msg import IOControl

#Create a class for a robolink joint
class robolinkJoint(object):
    maxAbsVelocity = 500.0  #motor max is 3000
    encoderResolution = 2048.0  #512cpr quadrature encoder
    Reduction = 318.0 #318:1 planetary reduction
    encoderCountsPerPulleyDegree = encoderResolution * Reduction/360.0
    def __init__(self):
        self.maxAngle = 0
        self.minAngle = 0
        self.homePos = 0
        self.controlMsg = EPOSControl()
        self.ioMsg = IOControl()
        self.info = GroupMotorInfo.motor_group
