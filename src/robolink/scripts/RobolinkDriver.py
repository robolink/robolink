#!/usr/bin/env python  
import roslib

roslib.load_manifest('robolink')
import rospy
import numpy
import math

from EposManager.msg import EPOSControl
from EposManager.msg import GroupEPOSControl
from EposManager.msg import GroupMotorInfo

from sensor_msgs.msg import Joy
from robolink.msg import RobolinkInfo
from robolink.msg import RobolinkControl
import tf

from robolink.classes import robolinkJoint
from robolink.classes import tipVelocity
from geometry_msgs.msg import Pose

import array
from array import array
import csv

groupDrivePublisher = rospy.Publisher('Group_Motor_Control', GroupEPOSControl)
RobolinkInfoPublisher = rospy.Publisher('Robolink_Info', RobolinkInfo)
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
METHODS
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
def poseControl(pose = RobolinkControl()):
    '''
    Take in a desired point message and servo toward it
    '''
    try:
        tfListener.waitForTransform('armBase', pose.header.frame_id, rospy.Time(), rospy.Duration(3.0))
              
        now = pose.header.stamp
        
        tfListener.waitForTransform('armBase', pose.header.frame_id, now, rospy.Duration(3.0))
        newPose = tfListener.transformPose('armBase', pose)
        
        tfListener.waitForTransform('armBase', 'gripper', now, rospy.Duration(3.0))
        (trans,rot) = tfListener.lookupTransform('armBase', 'gripper', now)
        print "Current x,y,z:" + str(trans[0]) + ", " + str(trans[1])+ ", " + str(trans[2])
        print "Desired x,y,z:" + str(newPose.pose.position.x) + ", " + str(newPose.pose.position.y)+ ", " + str(newPose.pose.position.z)
        
        #Convert the rotations from quaternions to euler angles
        (tipRotx, tipRoty, tipRotz) = tf.transformations.euler_from_quaternion(rot)
        (goalRotx, goalRoty, goalRotz) = tf.transformations.euler_from_quaternion([newPose.pose.orientation.x, newPose.pose.orientation.y, newPose.pose.orientation.z, newPose.pose.orientation.w])
     
        # An array to store the error between the tip and goal
        # x, y, z, roll, pitch, yaw
        tipError = array('d',[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
               
        tipError[0] = newPose.pose.position.x - trans[0] 
        tipError[1] = newPose.pose.position.y - trans[1]
        tipError[2] = newPose.pose.position.z - trans[2]
        
        tipError[3] = goalRotx - tipRotx 
        tipError[4] = goalRoty - tipRoty
        tipError[5] = goalRotz - tipRotz
        
        print "Error x,y,z:" + str(tipError[0]) + ", " + str(tipError[1])+ ", " + str(tipError[2])
        
        tipVelocity = PIDController(tipError)
        print "Tip Velocity x,y,z:" + str(tipVelocity[0]) + ", " + str(tipVelocity[1])+ ", " + str(tipVelocity[2])
        jointVelocities = convertTipVelocitiesToJointVelocities(tipVelocity)
        
        print "jointVelocities = " + str(jointVelocities[0]) + ", " + str(jointVelocities[1]) + ", " + str(jointVelocities[2]) + ", " + str(jointVelocities[3]) + ", " + str(jointVelocities[4])
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.ERROR(e)      
    
    setAllVelocities(int(jointVelocities[0]), int(jointVelocities[1]), int(jointVelocities[2]), int(jointVelocities[3]), int(jointVelocities[4]))
        
def generateHTM(a, alpha, d, theta):
    '''
    Generates an HTM given 4 DH parameters.
    NOTE: alpha and theta should be in degrees.
    '''
    
    a1 = cosd(theta)
    a2 = -sind(theta) * cosd(alpha)
    a3 = sind(theta) * sind(alpha)
    a4 = a * cosd(theta)
    
    b1 = sind(theta)
    b2 = cosd(theta) * cosd(alpha)
    b3 = -cosd(theta) * sind(alpha)
    b4 = a * sind(theta)
        
    c1 = 0
    c2 = sind(alpha)
    c3 = cosd(alpha)
    c4 = d
    
    d1 = 0
    d2 = 0
    d3 = 0
    d4 = 1
    
    HTM = numpy.matrix([[a1, a2, a3, a4],
                        [b1, b2, b3, b4],
                        [c1, c2, c3, c4],
                        [d1, d2, d3, d4]])
    return HTM

def updateDHTable():
    '''
    Recalculated the DH Table based on the current joint configuration
    '''
    #DH Parameters
    #[transx(a) rotx(alpha) transz(d) rotz(theta)]
    robolinkDH = numpy.matrix([[0.0, -90.0, 0.0, 0.0],
                               [0.0, 0.0, 0.0, 90+jointVariablesList[0]],
                               [0.0, -90.0, link1Length, 0.0],
                               [0.0, 90.0, 0.0, jointVariablesList[1]],
                               [0.0, -90.0, link2Length, jointVariablesList[2]],
                               [0.0, 90.0, 0.0, jointVariablesList[3]],
                               [0.0, 0.0, link3Length, jointVariablesList[4]]])
    return robolinkDH

def updateTFTree(robolinkDH):
    '''
    Updates and rebroadcasts the TFTree based on the most up to date DH-Table
    '''
    now = rospy.Time.now()
    
    armBase = generateHTM(robolinkDH[0,0], robolinkDH[0,1], robolinkDH[0,2], robolinkDH[0,3])
    rot = tf.transformations.quaternion_from_matrix(rotationMatrixFromHTM(armBase))
    tfBroadcaster.sendTransform(xyzFromHTM(armBase),rot,now,"armBase","baseFrame")
    
    joint0 = generateHTM(robolinkDH[1,0], robolinkDH[1,1], robolinkDH[1,2], robolinkDH[1,3])
    rot = tf.transformations.quaternion_from_matrix(rotationMatrixFromHTM(joint0))
    tfBroadcaster.sendTransform(xyzFromHTM(joint0),rot,now,"joint0","armBase")
    
    joint1 = generateHTM(robolinkDH[2,0], robolinkDH[2,1], robolinkDH[2,2], robolinkDH[2,3])
    rot = tf.transformations.quaternion_from_matrix(rotationMatrixFromHTM(joint1))
    tfBroadcaster.sendTransform(xyzFromHTM(joint1),rot,now,"joint1","joint0")
    
    joint2 = generateHTM(robolinkDH[3,0], robolinkDH[3,1], robolinkDH[3,2], robolinkDH[3,3])
    rot = tf.transformations.quaternion_from_matrix(rotationMatrixFromHTM(joint2))
    tfBroadcaster.sendTransform(xyzFromHTM(joint2),rot,now,"joint2","joint1")
    
    joint3 = generateHTM(robolinkDH[4,0], robolinkDH[4,1], robolinkDH[4,2], robolinkDH[4,3])
    rot = tf.transformations.quaternion_from_matrix(rotationMatrixFromHTM(joint3))
    tfBroadcaster.sendTransform(xyzFromHTM(joint3),rot,now,"joint3","joint2")
    
    joint4 = generateHTM(robolinkDH[5,0], robolinkDH[5,1], robolinkDH[5,2], robolinkDH[5,3])
    rot = tf.transformations.quaternion_from_matrix(rotationMatrixFromHTM(joint4))
    tfBroadcaster.sendTransform(xyzFromHTM(joint4),rot,now,"joint4","joint3")
    
    gripper = generateHTM(robolinkDH[6,0], robolinkDH[6,1], robolinkDH[6,2], robolinkDH[6,3])
    rot = tf.transformations.quaternion_from_matrix(rotationMatrixFromHTM(gripper))
    tfBroadcaster.sendTransform(xyzFromHTM(gripper),rot,now,"gripper","joint4")    
    
    
    
def updateJacobian():
    '''
    Updates the Jacobian based on the current joint angles
    '''
    
    q0 = jointVariablesList[0]
    q1 = jointVariablesList[1]
    q2 = jointVariablesList[2]
    q3 = jointVariablesList[3]
    q4 = jointVariablesList[4]
    
    a1 = ((-cosd(q0)*cosd(q1)*cosd(q2)+sind(q0)*sind(q2))*sind(q3)-cosd(q0)*sind(q1)*cosd(q3))*link2Length-cosd(q0)*sind(q1)*link1Length
    a2 = (sind(q0)*sind(q1)*cosd(q2)*sind(q3)-sind(q0)*cosd(q1)*cosd(q3))*link2Length-sind(q0)*cosd(q1)*link1Length
    a3 = (sind(q0)*cosd(q1)*sind(q2)-cosd(q0)*cosd(q2))*sind(q3)*link2Length
    a4 = ((-sind(q0)*cosd(q1)*cosd(q2)-cosd(q0)*sind(q2))*cosd(q3)+sind(q0)*sind(q1)*sind(q3))*link2Length
    a5 = 0
    
    b1 = ((-sind(q0)*cosd(q1)*cosd(q2)-cosd(q0)*sind(q2))*sind(q3)-sind(q0)*sind(q1)*cosd(q3))*link2Length-sind(q0)*sind(q1)*link1Length
    b2 = (-cosd(q0)*sind(q1)*cosd(q2)*sind(q3)+cosd(q0)*cosd(q1)*cosd(q3))*link2Length+cosd(q0)*cosd(q1)*link1Length
    b3 = (-cosd(q0)*cosd(q1)*sind(q2)-sind(q0)*cosd(q2))*sind(q3)*link2Length
    b4 = ((cosd(q0)*cosd(q1)*cosd(q2)-sind(q0)*sind(q2))*cosd(q3)-cosd(q0)*sind(q1)*sind(q3))*link2Length
    b5 = 0
    
    c1 = 0
    c2 = (-cosd(q1)*cosd(q2)*sind(q3)-sind(q1)*cosd(q3))*link2Length-sind(q1)*link1Length
    c3 = sind(q1)*sind(q2)*sind(q3)*link2Length
    c4 = (-sind(q1)*cosd(q2)*cosd(q3)-cosd(q1)*sind(q3))*link2Length
    c5 = 0
    
    d1 = 0
    d2 = -sind(q0)*sind(q1)
    d3 = sind(q0)*cosd(q1)*sind(q2)-cosd(q0)*cosd(q2)
    d4 = (-sind(q0)*cosd(q1)*cosd(q2)-cosd(q0)*sind(q2))*sind(q3)-sind(q0)*sind(q1)*cosd(q3)
    d5 = (-sind(q0)*cosd(q1)*cosd(q2)-cosd(q0)*sind(q2))*sind(q3)-sind(q0)*sind(q1)*cosd(q3)
    
    e1 = 0
    e2 = cosd(q0)*sind(q1)
    e3 = -cosd(q0)*cosd(q1)*sind(q2)-sind(q0)*cosd(q2)
    e4 = (cosd(q0)*cosd(q1)*cosd(q2)-sind(q0)*sind(q2))*sind(q3)+cosd(q0)*sind(q1)*cosd(q3)
    e5 = (cosd(q0)*cosd(q1)*cosd(q2)-sind(q0)*sind(q2))*sind(q3)+cosd(q0)*sind(q1)*cosd(q3)
        
    f1 = 1
    f2 = cosd(q1)
    f3 = sind(q1)*sind(q2)
    f4 = -sind(q1)*cosd(q2)*sind(q3)+cosd(q1)*cosd(q3)
    f5 = -sind(q1)*cosd(q2)*sind(q3)+cosd(q1)*cosd(q3)
    
    
    # because of the joint limitations on the arm, we're going to use position control only.  this means we only use the top half of the jacobian.
    global Jacobian 
    Jacobian = numpy.matrix([[a1, a2, a3, a4, a5],
                             [b1, b2, b3, b4, b5],
                             [c1, c2, c3, c4, c5],
                             [d1, d2, d3, d4, d5],
                             [e1, e2, e3, e4, e5],
                             [f1, f2, f3, f4, f5]])
    
    global PositionJacobian
    PositionJacobian = numpy.matrix([[a1, a2, a3, a4, a5],
                                     [b1, b2, b3, b4, b5],
                                     [c1, c2, c3, c4, c5]])
    
def convertTipVelocitiesToJointVelocities(tipVelocities):
    '''
    Takes in an array called tipVelocities [x,y,z,roll,pitch,yaw] in m/s and deg/s
    And outputs an array called jointVelocities [joint0, joint1, joint2, joint3, joint4] in deg/s
    '''
    #Ignore the rotation components because of joint limitations
    TipVelocitiesMatrix = numpy.matrix([[tipVelocities[0]],
                                        [tipVelocities[1]],
                                        [tipVelocities[2]]])
    
    JointVelocitiesMatrix = numpy.matrix([[0.0],
                                          [0.0],
                                          [0.0],
                                          [0.0],
                                          [0.0]])
    
    #calculate the psuedo inverse of the position jacobian
    #Jinv = numpy.linalg.pinv(Jacobian)
    PJinv = numpy.linalg.pinv(PositionJacobian)
    #Use the pesudo inverse to calculate the individual joint velocities
    JointVelocitiesMatrix = PJinv * TipVelocitiesMatrix
    #make the joint velocities easier to use when returned
    jointVelocities = array('d',[JointVelocitiesMatrix[0,0], JointVelocitiesMatrix[1,0], JointVelocitiesMatrix[2,0], JointVelocitiesMatrix[3,0], JointVelocitiesMatrix[4,0]])
    
    # the joint velocities are in degrees/second, convert these to rpm for the motor
    for i in range(len(jointVelocities)):
        jointVelocities[i] *= 60*robolinkJoint.Reduction/360
        #joint velcoities of less than 100 are barely noticeable and won't have any marked effect.
        if abs(jointVelocities[i]) < 10:
            jointVelocities[i] = 0
    
    return jointVelocities
            
def xyzFromHTM(HTM):
    '''
    Pulls x,y,z translation from an HTM
    '''
    x = HTM[0,3]
    y = HTM[1,3]
    z = HTM[2,3]
    return (x,y,z)

def rotationMatrixFromHTM(HTM):
    '''
    Pulls the rotation matrix from an HTM and pads it to make it a 4x4 matrix
    '''
    rot = numpy.matrix([[HTM[0,0], HTM[0,1], HTM[0,2], 0],
                        [HTM[1,0], HTM[1,1], HTM[1,2], 0],
                        [HTM[2,0], HTM[2,1], HTM[2,2], 0],
                        [0, 0, 0, 1]])
    return rot

def sind(theta):
    '''
    simple way of doing sine in degrees
    '''
    val = math.sin(math.radians(theta))
    
    #this ignores an error in python's approximation of sine
    if(abs(val) < 0.000001):
        val = 0;
    
    return val

def cosd(theta):
    '''
    simple way of doing cosine in degrees
    '''
    val = math.cos(math.radians(theta))
    
    #this ignores an error in python's approximation of cosine
    if(abs(val) < 0.000001):
        val = 0;
    
    return val

def PIDController(tipError):
    tipVelocity = array('d',[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # in a for loop, iterate through x, y, z, roll, pitch, and yaw
    for i in range(len(tipError)):
        error = tipError[i]
        P = Pconstants[i]
        I = Iconstants[i]
        D = Dconstants[i]
        prevError = prevErrors[i]
        cumulativeError = cumulativeErrors[i]
        
        tipVelocity[i] = P*error + I*cumulativeError + D*prevError
        
        prevErrors[i] = error
        
        if error == 0:
            cumulativeErrors[i] = 0
        else:                
            #TODO: add integral error limit
            cumulativeErrors[i] += error    
    
    return tipVelocity

def convertEncoderCountsToDegrees(jointNum, counts):
    """
    Consumes counts, looks up the conversion for the specified joint number,
    and performs the conversion.
    """
    degrees = int(counts/jointList[jointNum].encoderCountsPerPulleyDegree)
    return degrees  

def convertDegreesToEncoderCounts(jointNum, degrees):
    """
    Consumes degrees, looks up the conversion for the specified joint number,
    and performs the conversion.
    """
    counts = int(degrees*jointList[jointNum].encoderCountsPerPulleyDegree)
    return counts

def checkJointsForMaxTravel(jointVelocities):
    '''
    Check the joints to make sure they aren't at their maximum angle.
    If they are, make sure velocities in that direction are 0
    '''
    for joint in range(len(jointList)):
        if(convertEncoderCountsToDegrees(joint, jointList[joint].info.motor_position) < jointList[joint].minAngle and jointVelocities[joint] <= 0):
            #print jointList[joint].info.motor_na " is at its minimum angle of " + str(convertEncoderCountsToDegrees(joint, jointList[joint].info.motor_position)) + " degrees."
            jointVelocities[joint] = 0
            rospy.logwarn("Joint %s has reached is minimum travel.", str(joint))
        if(convertEncoderCountsToDegrees(joint, jointList[joint].info.motor_position) > jointList[joint].maxAngle and jointVelocities[joint] >= 0):
            #print jointList[joint].info.motor_name + " is at its maximum angle of " + str(convertEncoderCountsToDegrees(joint, jointList[joint].info.motor_position)) + " degrees."
            jointVelocities[joint] = 0
            rospy.logwarn("Joint %s has reached is maximum travel.", str(joint))
            
    return jointVelocities

def initialize():
    
    global timeLastMsgReceived
    global watchdogDuration
    global watchdogErrorPrinted
    watchdogDuration = 3
    watchdogErrorPrinted = False
        
    #Link Lengths in meters 
    global link1Length
    link1Length = 0.28575
    
    global link2Length
    link2Length = .45085
    
    global link3Length
    link3Length = .45085
    
    global link4Length
    link4Length = 0.0##50.0
    
    #define each joint as an robolink joint.
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
    global jointList
    jointList = [joint0, joint1, joint2, joint3, joint4]
    
    #joint variables
    q0 = 0.0
    q1 = 0.0
    q2 = 0.0
    q3 = 0.0
    q4 = 0.0
    
    #Put these joint variables in a list to make referencing them easier
    global jointVariablesList
    jointVariablesList = [q0, q1, q2, q3, q4]
    
    global jointVelocitiesList
    jointVelocitiesList = array('f',[0.0, 0.0, 0.0, 0.0, 0.0])
    
    #DH Parameters
    #[transx(a) rotx(alpha) transz(d) rotz(theta)]
    robolinkDH = numpy.matrix([[0.0, 90.0, 0.0, 0.0],
                               [0.0, 90.0, link1Length, 90-jointVariablesList[0]],
                               [0.0, -90.0, 0.0, jointVariablesList[1]],
                               [0.0, 90.0, link2Length, jointVariablesList[2]],
                               [0.0, -90.0, 0.0, jointVariablesList[3]],
                               [0.0, 90.0, link3Length, jointVariablesList[4]],
                               [link4Length, 90.0, 0.0, 0.0]])
    
    #Jacobian
    '''
    global Jacobian
    Jacobian = numpy.matrix([[0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0]])
    '''
    #PID constants and storage arrays
    global Pconstants
    Pconstants = array('d',[20.0, 20.0, 20.0, 20.0, 20.0, 20.0])
    
    global Iconstants
    Iconstants = array('d',[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    global Dconstants
    Dconstants = array('d',[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    global prevErrors
    prevErrors = array('d',[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    global cumulativeErrors
    cumulativeErrors = array('d',[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    global tfListener
    tfListener = tf.TransformListener()
    
    global tfBroadcaster
    tfBroadcaster = tf.TransformBroadcaster()
        
    setAllVelocities(0, 0, 0, 0, 0)
    
def resetWatchdog(stamp):
    #set the watchdog timer
    global timeLastMsgReceived
    timeLastMsgReceived = stamp
    global watchdogErrorPrinted
    if watchdogErrorPrinted:
        rospy.loginfo("Control message received. Watchdog reset.")
        watchdogErrorPrinted = False

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
MOTOR CONTROL METHODS
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
def setAllVelocities(joint0_velocity, joint1_velocity, joint2_velocity, joint3_velocity, joint4_velocity):
    """
    Sets the desired velocities and publishes the control message
    """
    #The message to be used to control all of the motors
    GroupMsg = GroupEPOSControl()
    
    #An array to store all of the motor velocities
    #An array to store all of the motor velocities
    global velocitySetpoints
    velocitySetpoints = array('l', [joint0_velocity, joint1_velocity, joint2_velocity, joint3_velocity, joint4_velocity])
    
    for i in range(len(jointList)):
        
        jointList[i].controlMsg.control_mode = EPOSControl.VELOCITY
        jointList[i].controlMsg.setpoint = velocitySetpoints[i]
        GroupMsg.motor_group.append(jointList[i].controlMsg)
    
    groupDrivePublisher.publish(GroupMsg)

def setAllAbsolutePositions(joint0_pos, joint1_pos, joint2_pos, joint3_pos, joint4_pos):
    """
    Sets the desired absolute positions and publishes the control message
    """
    #The message to be used to control all of the motors
    GroupMsg = GroupEPOSControl()
    
    #A global array to store all of the motor absolute positions
    global absolutePositionSetpoints
    absolutePositionSetpoints = array('l', [joint0_pos, joint1_pos, joint2_pos, joint3_pos, joint4_pos])
    
    for i in range(len(jointList)):
        jointList[i].controlMsg.control_mode = EPOSControl.ABSOLUTE_POSITION_IMMEDIATE
        jointList[i].controlMsg.setpoint = absolutePositionSetpoints[i]
        GroupMsg.motor_group.append(jointList[i].controlMsg)
        
    groupDrivePublisher.publish(GroupMsg)
    
def setAllRelativePositions(joint0_pos, joint1_pos, joint2_pos, joint3_pos, joint4_pos):
    """
    Sets the desired relative positions and publishes the control message
    """
    #A global array to store all of the motor relative positions
    relativePositionSetpoints = array('l', [joint0_pos, joint1_pos, joint2_pos, joint3_pos, joint4_pos])

    #The message to be used to control all of the motors
    GroupMsg = GroupEPOSControl()
    
    for i in range(len(jointList)):
        jointList[i].controlMsg.control_mode = EPOSControl.RELATIVE_POSITION_IMMEDIATE
        jointList[i].controlMsg.setpoint = relativePositionSetpoints[i]
        GroupMsg.motor_group.append(jointList[i].controlMsg)
        
    #Publish the message             
    groupDrivePublisher.publish(GroupMsg)

def setSingleVelocity(motorNum, velocity, debug=False):
    """
    Sets the velocity of one motor and publishes the group control message
    """
    if debug == True:
        rospy.logdebug("Moving joint %s at %s", str(motorNum), str(velocity))
        
    #The message to be used to control all of the motors
    GroupMsg = GroupEPOSControl()
    
    jointList[motorNum].controlMsg.control_mode = EPOSControl.VELOCITY
    jointList[motorNum].controlMsg.setpoint = velocity
    
    for i in range(len(jointList)):
        GroupMsg.motor_group.append(jointList[i].controlMsg)
        
    #Publish the message
    groupDrivePublisher.publish(GroupMsg)
    
def setSingleAbsolutePosition(motorNum, pos, debug=False):
    """
    Sets the absolute position of one motor and publishes the group control message
    """
    if debug == True:
        rospy.logdebug("Moving joint %s to %s", str(motorNum), str(velocity))
        
    #The message to be used to control all of the motors
    GroupMsg = GroupEPOSControl()
    
    jointList[motorNum].controlMsg.control_mode = EPOSControl.ABSOLUTE_POSITION_IMMEDIATE
    jointList[motorNum].controlMsg.setpoint = pos
    
    for i in range(len(jointList)):
        GroupMsg.motor_group.append(jointList[i].controlMsg)
        
    #Publish the message
    groupDrivePublisher.publish(GroupMsg)

def setSingleRelativePosition(motorNum, pos, debug=False):
    """
    Sets the relative position of one motor and publishes the group control message
    """
    if debug == True:
        rospy.logdebug("Moving joint %s %s counts", str(motorNum), str(pos))
        
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
            rospy.logdebug("Waiting for joint to achieve %s currently at %s.", str(target), str(jointList[motorNum].info.motor_position))

def setSingleRelativeAngle(motorNum, degrees, debug=False):
    """
    Sets the relative angle of a joint in degrees
    """
    if debug == True:
        rospy.logdebug("Moving joint %s %s degrees", str(motorNum), str(pos))
        
    setSingleRelativePosition(motorNum, convertDegreesToEncoderCounts(motorNum, degrees),debug)

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
CALLBACKS
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
def robolinkControlCallback(msg = RobolinkControl()):
    resetWatchdog(msg.header.stamp)
    
    if msg.control_mode == RobolinkControl.JOINT_ABSOLUTE_POSITION:
        
        setAllAbsolutePositions(msg.joint_0_setpoint, msg.joint_1_setpoint, msg.joint_2_setpoint, msg.joint_3_setpoint, msg.joint_4_setpoint)
        
    elif msg.control_mode == RobolinkControl.JOINT_RELATIVE_POSITION:
        
        setAllRelativePositions(msg.joint_0_setpoint, msg.joint_1_setpoint, msg.joint_2_setpoint, msg.joint_3_setpoint, msg.joint_4_setpoint)
        
    elif msg.control_mode == RobolinkControl.JOINT_VELOCITY:
        
        jointVelocities = [msg.joint_0_setpoint, msg.joint_1_setpoint, msg.joint_2_setpoint, msg.joint_3_setpoint, msg.joint_4_setpoint]        
        jointVelocities = checkJointsForMaxTravel(jointVelocities)        
        setAllVelocities(jointVelocities[0], jointVelocities[1], jointVelocities[2], jointVelocities[3], jointVelocities[4])
        
    elif msg.control_mode == RobolinkControl.POSE_CONTROL:
        
        poseControl(msg)
        
    elif msg.control_mode == RobolinkControl.TWIST_CONTROL:
        
        setAllVelocities(0, 0, 0, 0, 0)
        rospy.ERROR("Twist control not yet implemented.  Stopping all joints.")
    else:
        
        setAllVelocities(0, 0, 0, 0, 0)
        rospy.ERROR("Invalid Robolink control_mode.  Stopping all joints.")
        
def motorInfoCallback(msg):
    '''
    Read the message into joint.info
    '''
    for i in range(len(msg.motor_group)):
        
        jointList[i].info = msg.motor_group[i]
        jointVariablesList[i] = jointList[i].info.motor_position/jointList[i].encoderCountsPerPulleyDegree
        jointVelocitiesList[i] = jointList[i].info.motor_velocity/jointList[i].encoderCountsPerPulleyDegree
    
    dh = updateDHTable()   
    updateTFTree(dh)
    
    #update the jabcobian with the most recent joint configuration
    updateJacobian()
    
def publishRobolinkInfoCallback(event):
    
    msg = RobolinkInfo()
    #wait for the joints to have an info attribute
    if hasattr(jointList[0].info, 'motor_name'): 
        currentPos = Pose()
        now = rospy.Time.now()
        tfListener.waitForTransform('armBase', 'gripper', now, rospy.Duration(3.0))
        (trans,rot) = tfListener.lookupTransform('armBase', 'gripper', now)
        currentPos.position.x = trans[0]
        currentPos.position.y = trans[1]
        currentPos.position.z = trans[2]
        currentPos.orientation.w = rot[0]
        currentPos.orientation.x = rot[1]
        currentPos.orientation.y = rot[2]
        currentPos.orientation.z = rot[3]
        msg.current_position = currentPos
        msg.stamp = rospy.Time.now()
        for i in range(len(jointList)):            
            msg.joint_angles.append(convertEncoderCountsToDegrees(i, jointList[i].info.motor_position))            
            msg.joint_velocities.append(jointList[i].info.motor_velocity/robolinkJoint.Reduction)
            
        RobolinkInfoPublisher.publish(msg)
        
        
        '''Uncomment to enable csv logging
        anglesFile = open("/home/ttremblay/JointAngles.csv",'a')
        wr = csv.writer(anglesFile, dialect='excel')
        wr.writerow(msg.joint_angles)
        anglesFile.close()

        velocitiesFile = open("/home/ttremblay/JointVelocities.csv",'a')
        wr = csv.writer(velocitiesFile, dialect='excel')
        wr.writerow(msg.joint_velocities)
        velocitiesFile.close()
        '''
        
def monitorOverTravelCallback(event):
    '''
    Periodically checks to see if the joints are traveling too far because
    the joyCallback only gets called when there is a change in the joy message.
    '''
    #wait for the joints to have an info attribute
    if hasattr(jointList[0].info, 'motor_name'):
        jointVelocities = velocitySetpoints
        
        jointVelocities = checkJointsForMaxTravel(jointVelocities)
        
        setAllVelocities(jointVelocities[0], jointVelocities[1], jointVelocities[2], jointVelocities[3], jointVelocities[4])    

def watchdogCallback(event):
    '''
    if it's been more than 3 seconds since the last control message was received, stop the motors
    '''
    now = rospy.Time.now()
    if (now-timeLastMsgReceived) > rospy.Duration(watchdogDuration):
        setAllVelocities(0, 0, 0, 0, 0)
        global watchdogErrorPrinted
        if not watchdogErrorPrinted:
            rospy.logerr("Robolink control message not received within the last %s seconds.  Stopping all motors. This message will only print once.", str(watchdogDuration))
            watchdogErrorPrinted = True
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
MAIN
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""    
if __name__ == '__main__':
    try:
        rospy.init_node('RobolinkDriver', anonymous=True)
        initialize()
        timeLastMsgReceived = rospy.Time.now()
        rospy.Subscriber("Group_Motor_Info", GroupMotorInfo, motorInfoCallback)
        rospy.Subscriber("Robolink_Control", RobolinkControl, robolinkControlCallback)
        rospy.Timer(rospy.Duration(0.1), publishRobolinkInfoCallback)
        rospy.Timer(rospy.Duration(0.1), monitorOverTravelCallback)
        rospy.Timer(rospy.Duration(0.1), watchdogCallback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
    finally:
        setAllVelocities(0,0,0,0,0)
