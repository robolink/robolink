#!/usr/bin/env python
import roslib; 
import ros
roslib.load_manifest('robolink')
import rospy

from EposManager.msg import EPOSControl
from EposManager.msg import GroupEPOSControl
from EposManager.msg import GroupMotorInfo
from sensor_msgs.msg import Joy
from robolink.msg import GroupRobolinkJointInfo
from robolink.msg import RobolinkJointInfo

import sys, select, termios, tty, array
from array import array

from robolink.classes import robolinkJoint

groupDrivePublisher = rospy.Publisher('Group_Motor_Control', GroupEPOSControl)
groupRobolinkJointInfoPublisher = rospy.Publisher('Group_Robolink_Joint_Info', GroupRobolinkJointInfo)

#digitalOutputPublisher = rospy.Publisher('Digital_Output_Control', IOControl)

#define each joint as an robolink joint.
ioController = robolinkJoint()
joint0 = robolinkJoint()
joint1 = robolinkJoint()
joint2 = robolinkJoint()
joint3 = robolinkJoint()
joint4 = robolinkJoint()

#Set the maximum absolute position in degrees for each joint
joint0.maxAngle = rospy.get_param('/joint_0/Max_Angle', 180)
joint0.minAngle = rospy.get_param('/joint_0/Min_Angle', -180)

joint1.maxAngle = rospy.get_param('/joint_1/Max_Angle', 130)
joint1.minAngle = rospy.get_param('joint_1/Min_Angle', -50)

joint2.maxAngle = rospy.get_param('/joint_2/Max_Angle', 180)
joint2.minAngle = rospy.get_param('/joint_2/Min_Angle', -180)

joint3.maxAngle = rospy.get_param('/joint_3/Max_Angle', 90)
joint3.minAngle = rospy.get_param('/joint_3/Min_Angle', -90)

joint4.maxAngle = rospy.get_param('/joint_4/Max_Angle', 180)
joint4.minAngle = rospy.get_param('/joint_4/Min_Angle', -180)

#Set the home position of each joint.  This is the encoder position that results in a 0 degree joint position
joint0.homePos = rospy.get_param('/joint_0/Home_Pos', 0)
joint1.homePos = rospy.get_param('/joint_1/Home_Pos', 0)
joint2.homePos = rospy.get_param('/joint_2/Home_Pos', 0)
joint3.homePos = rospy.get_param('/joint_3/Home_Pos', 0)
joint4.homePos = rospy.get_param('/joint_4/Home_Pos', 0)


#CAN Node IDs for the motors as determined by EPOS Studio.
#Correlation between DIP switch on motherboard and resulting
#Node ID is not known.  
joint0.controlMsg.node_id = rospy.get_param('/joint_0/Node_ID', 2)
joint1.controlMsg.node_id = rospy.get_param('/joint_1/Node_ID', 3)       
joint2.controlMsg.node_id = rospy.get_param('/joint_2/Node_ID', 16)
joint3.controlMsg.node_id = rospy.get_param('/joint_3/Node_ID', 8)
joint4.controlMsg.node_id = rospy.get_param('/joint_4/Node_ID', 4)

#Put these joints in a list to make referencing them easier
jointList = [joint0, joint1, joint2, joint3, joint4]

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
SET METHODS
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
def setAllVelocities(joint0_velocity, joint1_velocity, joint2_velocity, joint3_velocity, joint4_velocity):
    """
    Sets the desired velocities and publishes the control message
    """
    #The message to be used to control all of the motors
    GroupMsg = GroupEPOSControl()
    
    #An array to store all of the motor velocities
    velocitySetpoints = array('l', [0, 0, 0, 0, 0])

    velocitySetpoints[0] = joint0_velocity
    velocitySetpoints[1] = joint1_velocity
    velocitySetpoints[2] = joint2_velocity
    velocitySetpoints[3] = joint3_velocity
    velocitySetpoints[4] = joint4_velocity
    
    for i in range(len(jointList)):
        jointList[i].controlMsg.control_mode = EPOSControl.VELOCITY
        jointList[i].controlMsg.setpoint = velocitySetpoints[i]
        GroupMsg.motor_group.append(jointList[i].controlMsg)
    
    groupDrivePublisher.publish(GroupMsg)
    
def setAllRelativePositions(joint0_pos, joint1_pos, joint2_pos, joint3_pos, joint4_pos):
    """
    Sets the desired relative positions and publishes the control message
    """
    #A global array to store all of the motor relative positions
    relativePositionSetpoints = array('l', [0, 0, 0, 0, 0])

    #The message to be used to control all of the motors
    GroupMsg = GroupEPOSControl()
    
    relativePositionSetpoints[0] = joint0_pos
    relativePositionSetpoints[1] = joint1_pos
    relativePositionSetpoints[2] = joint2_pos
    relativePositionSetpoints[3] = joint3_pos
    relativePositionSetpoints[4] = joint4_pos
    
    for i in range(len(jointList)):
        jointList[i].controlMsg.control_mode = EPOSControl.ABSOLUTE_POSITION_IMMEDIATE
        jointList[i].controlMsg.setpoint = absolutePositionSetpoints[i]
        GroupMsg.motor_group.append(jointList[i].controlMsg)
        
    #Publish the message             
    groupDrivePublisher.publish(GroupMsg)

def setSingleVelocity(motorNum, velocity, debug=False):
    """
    Sets the velocity of one motor and publishes the group control message
    """
    if debug == True:
        print "Moving joint" + str(motorNum) + " at " + str(velocity)
        
    #The message to be used to control all of the motors
    GroupMsg = GroupEPOSControl()
    
    jointList[motorNum].controlMsg.control_mode = EPOSControl.VELOCITY
    jointList[motorNum].controlMsg.setpoint = velocity
    
    for i in range(len(jointList)):
        GroupMsg.motor_group.append(jointList[i].controlMsg)
        
    #Publish the message
    groupDrivePublisher.publish(GroupMsg)

def setSingleRelativePosition(motorNum, pos, debug=False):
    """
    Sets the relative position of one motor and publishes the group control message
    """
    if debug == True:
        print "Moving joint" + str(motorNum) + " " +  str(pos) + " counts."
        
    #The message to be used to control all of the motors
    GroupMsg = GroupEPOSControl()
    
    #Save the previous position so you can make sure you're on target before leaving
    prevPos = jointList[motorNum].info.motor_position
    target = prevPos + pos
    
    jointList[motorNum].controlMsg.control_mode = EPOSControl.RELATIVE_POSITION_IMMEDIATE
    jointList[motorNum].controlMsg.setpoint = pos
    
    for i in range(len(jointList)):
        GroupMsg.motor_group.append(jointList[i].controlMsg)
        
    #Publish the message
    groupDrivePublisher.publish(GroupMsg)
    
    #Wait for the joint to achieve that position
    while(jointList[motorNum].info.motor_position != target):
        if debug == True:
            print "waiting for joint to achieve " + str(target) + " currently at " + str(jointList[motorNum].info.motor_position)

def setSingleRelativeAngle(motorNum, degrees, debug=False):
    """
    Sets the relative angle of a joint in degrees
    """
    if debug == True:
        print "Moving joint" + str(motorNum) + " " + str(degrees) + " degrees."
        
    setSingleRelativePosition(motorNum, convertDegreesToEncoderCounts(motorNum, degrees),debug)
    
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
PUBLISH METHODS
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
def publishJointInfo(event):
    '''
    Builds the GroupRobolinkJointInfo message and publishes it for debugging purposes
    '''
    GroupMsg = GroupRobolinkJointInfo()
    for i in range(len(jointList)):
        msg = RobolinkJointInfo()
        msg.joint_name = jointList[i].info.motor_name
        msg.joint_num = i
        msg.joint_angle = convertEncoderCountsToDegrees(i, jointList[i].info.motor_position)
        msg.joint_velocity = jointList[i].info.motor_velocity/robolinkJoint.Reduction
        msg.stamp = rospy.Time.now()
        GroupMsg.joint_group.append(msg)
        
    groupRobolinkJointInfoPublisher.publish(GroupMsg)
    
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
CALLBACKS
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
def motorInfoCallback(msg):
    '''
    Read the message into joint.info
    '''
    for i in range(len(msg.motor_group)):
        
        jointList[i].info = msg.motor_group[i]
    
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
MISC METHODS
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
def convertDegreesToEncoderCounts(jointNum, degrees):
    """
    Consumes degrees, looks up the conversion for the specified joint number,
    and performs the conversion.
    """
    counts = int(degrees*jointList[jointNum].encoderCountsPerPulleyDegree)
    return counts
    
def convertEncoderCountsToDegrees(jointNum, counts):
    """
    Consumes counts, looks up the conversion for the specified joint number,
    and performs the conversion.
    """
    degrees = int(counts/jointList[jointNum].encoderCountsPerPulleyDegree)
    return degrees  

def robolinkHomingProcedure():
    print "Robolink homing procedure:"
    print "Ensure that the robolink is free to move and that all joints are at their zero state."
    raw_input("When ready, press [Enter]")

    print "Now Homing Joint 0"
    homeJoint(0)
    print "Joint 0 homed, setting parameter to " + str(jointList[0].info.motor_position)
    rospy.set_param("/joint_0/Home_Pos", jointList[0].info.motor_position)
    print "Successfully set parameter to " + str(rospy.get_param('/joint_0/Home_Pos', 0))

    print "Now Homing Joint 1"
    homeJoint(1)
    print "Joint 1 homed, setting parameter to " + str(jointList[1].info.motor_position)
    rospy.set_param("/joint_1/Home_Pos", jointList[1].info.motor_position)
    print "Successfully set parameter to " + str(rospy.get_param('/joint_1/Home_Pos', 0))

    print "Now Homing Joint 2"
    homeJoint(2)
    print "Joint 2 homed, setting parameter to " + str(jointList[2].info.motor_position)
    rospy.set_param("/joint_2/Home_Pos", jointList[2].info.motor_position)
    print "Successfully set parameter to " + str(rospy.get_param('/joint_2/Home_Pos', 0))
    
    print "Now Homing Joint 3"
    homeJoint(3)
    print "Joint 3 homed, setting parameter to " + str(jointList[3].info.motor_position)
    rospy.set_param("/joint_3/Home_Pos", jointList[3].info.motor_position)
    print "Successfully set parameter to " + str(rospy.get_param('/joint_3/Home_Pos', 0))
    
    print "Now Homing Joint 4"
    homeJoint(4)
    print "Joint 4 homed, setting parameter to " + str(jointList[4].info.motor_position)
    rospy.set_param("/joint_4/Home_Pos", jointList[4].info.motor_position)
    print "Successfully set parameter to " + str(rospy.get_param('/joint_4/Home_Pos', 0))
    
    print "Homing Complete.  Exiting..."
    
def homeJoint(joint):
    """
    Moves desired joint -10 degrees and then slowly drives the joint back until it sees the home signal
    """
    relativePositionSetpoints = [0, 0, 0, 0, 0]
    setSingleRelativeAngle(joint, -10)
    while(jointList[joint].info.digital_input_3_state == True):
        setSingleVelocity(joint, 200)
    setSingleVelocity(joint, 0)   
                
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
MAIN
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""    
# The main function
if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        rospy.init_node('robolinkMotors')        
        #rospy.Subscriber("joy", Joy, joyCallback)        
        rospy.Subscriber("Group_Motor_Info", GroupMotorInfo, motorInfoCallback)
        setAllVelocities(0, 0, 0, 0, 0)
        rospy.Timer(rospy.Duration(0.1), publishJointInfo)
        robolinkHomingProcedure()
    except rospy.ROSInterruptException:
        pass
    
    finally:
        setAllVelocities(0,0,0,0,0)