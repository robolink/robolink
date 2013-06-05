#!/usr/bin/env python
import roslib; 
import ros
roslib.load_manifest('robolink')
import rospy

from sensor_msgs.msg import Joy

from robolink.msg import RobolinkControl
from robolink.msg import RobolinkInfo

import tf

import array
from array import array

from robolink.classes import robolinkJoint

desiredPose = RobolinkControl()
currentPose = RobolinkInfo()

def zeroorientation(dpose):
    dpose.pose.orientation.x = 0
    dpose.pose.orientation.y = 0
    dpose.pose.orientation.z = 0
    dpose.pose.orientation.w = 1
desiredPosePublisher = rospy.Publisher('Robolink_Control', RobolinkControl)
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
CALLBACKS
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
def publishJointInfoCallback(event):
    '''
    Builds the GroupRobolinkJointInfo message and publishes it for debugging purposes
    '''
    GroupMsg = GroupRobolinkJointInfo()
    #wait for the joints to have an info attribute
    if hasattr(jointList[0].info, 'motor_name'): 
        for i in range(len(jointList)):
            msg = RobolinkJointInfo()
            msg.joint_name = jointList[i].info.motor_name
            msg.joint_num = i
            msg.joint_angle = convertEncoderCountsToDegrees(i, jointList[i].info.motor_position)
            msg.joint_velocity = jointList[i].info.motor_velocity/robolinkJoint.Reduction
            msg.stamp = rospy.Time.now()
            GroupMsg.joint_group.append(msg)
        
            groupRobolinkJointInfoPublisher.publish(GroupMsg)

def joyCallback(Joy):
               
    Abutton = Joy.buttons[0]
    Bbutton = Joy.buttons[1]
    Xbutton = Joy.buttons[2]
    Ybutton = Joy.buttons[3]
    PowerButton = Joy.buttons[8]
    StartButton = Joy.buttons[7]
    
    stepSize = 0.1
    now = rospy.Time.now()
    
    #desired flow:
    # (for now, hit "back" on the xbox to set the initial poisition)
    # - button pressed, move tip to desired joystick position "initialpositions" + x,y,z
    # - on release, move button back to initial position
    
    #log debuggggggg
    if Abutton:
        desiredPose.header.stamp = now
        desiredPose.pose.position.x = (Abutton*-stepSize)
        desiredPose.pose.position.y = 0
        desiredPose.pose.position.z = 0 
        zeroorientation(desiredPose)
        rospy.loginfo(dir(currentPose.current_position.position))
        rospy.loginfo("\nChanged pose to: \n%s", str(desiredPose.pose.position))
    if Ybutton:
        desiredPose.header.stamp = now
        desiredPose.pose.position.x = (Ybutton*stepSize)
        desiredPose.pose.position.y = 0
        desiredPose.pose.position.z = 0 
        zeroorientation(desiredPose)
        
        rospy.loginfo("\nChanged pose to: \n%s", str(desiredPose.pose.position))
        
#     if Abutton or Bbutton or Xbutton or Ybutton:        
#         stepSize = 0.050
#         
#         now = rospy.Time.now()
# 
#         desiredPose.header.stamp = now
#         desiredPose.pose.position.x = (Ybutton*stepSize)+(Abutton*-stepSize)
#         desiredPose.pose.position.y = (Bbutton*stepSize)+(Xbutton*-stepSize) 
#         desiredPose.pose.position.z = 0 
#         desiredPose.pose.orientation.x = 0
#         desiredPose.pose.orientation.y = 0
#         desiredPose.pose.orientation.z = 0
#         desiredPose.pose.orientation.w = 1
#         
#         rospy.loginfo("\nChanged pose to: \n%s", str(desiredPose.pose.position))
#         
#     elif StartButton:
#          
#         now = rospy.Time.now()
#  
#         desiredPose.header.stamp = now
#         desiredPose.pose.position.x = 0
#         desiredPose.pose.position.y = 0
#         desiredPose.pose.position.z = 0
#         desiredPose.pose.orientation.x = 0
#         desiredPose.pose.orientation.y = 0
#         desiredPose.pose.orientation.z = 0
#         desiredPose.pose.orientation.w = 1
#          
#         rospy.loginfo("\nChanged pose to: \n%s", str(desiredPose.pose.position))
        
    else:
         
        now = rospy.Time.now()
 
        desiredPose.header.stamp = now
        desiredPose.pose.position.x = 0
        desiredPose.pose.position.y = 0
        desiredPose.pose.position.z = 0
        desiredPose.pose.orientation.x = 0
        desiredPose.pose.orientation.y = 0
        desiredPose.pose.orientation.z = 0
        desiredPose.pose.orientation.w = 1
        
        rospy.loginfo("\nChanged pose to: \n%s", str(desiredPose.pose.position))
def publishDesiredPoseCallback(event):
    '''
    Publish the global desired pose message
    '''
    #Header
    desiredPose.header.frame_id = 'gripper'
    now = rospy.Time.now()
    desiredPose.header.stamp = now
    
    #Control Mode
    desiredPose.control_mode = RobolinkControl.POSE_CONTROL
    
    desiredPosePublisher.publish(desiredPose)
    
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
MISC METHODS
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
def initialize():        
    global tfListener
    tfListener = tf.TransformListener()
    
    now = rospy.Time.now()

    desiredPose.header.stamp = now
    desiredPose.pose.position.x = 0
    desiredPose.pose.position.y = 0
    desiredPose.pose.position.z = 0
    desiredPose.pose.orientation.x = 0
    desiredPose.pose.orientation.y = 0
    desiredPose.pose.orientation.z = 0
    desiredPose.pose.orientation.w = 1
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
MAIN
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""    
# The main function
if __name__ == '__main__':
    
    global initialpositions 
    initialpositions = (currentPose.current_position.position.x,currentPose.current_position.position.y,currentPose.current_position.position.z)
    try:
        rospy.init_node('DesiredPose')
        initialize()      
        rospy.Subscriber("joy", Joy, joyCallback)
        rospy.Timer(rospy.Duration(0.1), publishDesiredPoseCallback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
    finally:
        setAllVelocities(0,0,0,0,0)