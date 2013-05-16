#!/usr/bin/env python  
import roslib

roslib.load_manifest('robolink')
import rospy
        
class translationVelocity(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

class rotationVelocity(object):
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

class tipVelocity(object):
    def __init__(self):
        self.translationVelocity = translationVelocity()
        self.rotationVelocity = rotationVelocity()         