#!/usr/bin/env python
import roslib; 
import ros
roslib.load_manifest('robolink')
import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import String

from robolink.msg import RobolinkControl
from robolink.msg import RobolinkInfo

import tf

import array
from array import array

from robolink.classes import robolinkJoint

desiredPose = RobolinkControl()
currentPose = RobolinkInfo()
desiredPosePublisher = rospy.Publisher('Robolink_Control', RobolinkControl)
### Methods
METHOD_POSE  = "pose."
METHOD_TWIST = "twist."
METHOD_JOINT = None
###Method Sub Maps
MULT = 50

#pose
METHOD_SUB_MAP_POS = ['position.x','position.y','position.z']
METHOD_SUB_MAP_OR = ['orientation.x','orientation.y','orientation.z','orientation.w']
#twist
METHOD_SUB_MAP_LINEAR = ['linear.x','linear.y','linear.z']
METHOD_SUB_MAP_ANGULAR = ['angular.x','angular.y','angular.z']
#None
METHOD_SUB_MAP_JOINTS = ['joint_0_setpoint','joint_1_setpoint','joint_2_setpoint','joint_3_setpoint','joint_4_setpoint']

METHOD_SUB_MAP_POSE = METHOD_SUB_MAP_POS + METHOD_SUB_MAP_OR
METHOD_SUB_MAP_TWIST = METHOD_SUB_MAP_LINEAR + METHOD_SUB_MAP_ANGULAR


# control_modes (pulled out of robolink control msg)
JOINT_VELOCITY = 1
JOINT_ABSOLUTE_POSITION = 2
JOINT_RELATIVE_POSITION = 3
POSE_CONTROL = 4
TWIST_CONTROL = 5

global current_info
    
#super useful
def rec_getattr(obj, attr):
    return reduce(getattr, attr.split("."), obj)
def rec_setattr(obj, attr, value):
    attrs = attr.split(".")
    setattr(reduce(getattr, attrs[:-1], obj), attrs[-1], value)
def negate(l):
    return [-x for x in l]

class CommandMessage(object):
    def __init__(self, positions=[], timeinterval=[], timelength=0.5, reset=True, startingstep=0, steps=[], controller=desiredPose, method="pose.", method_sub_map =METHOD_SUB_MAP_POS, control_mode=POSE_CONTROL):
        """ ARGS: 
        
            position:     List; of positions
            timeinterval: List; of times to idle between positions
            timelength:   Int;  Overrides the default timelength that is passed into the function when generating a timeinterval
            reset:        Bool; --go back to where when started (useful for driving, less for controlling light switches)
            startingstep: Int ; --Have a list of positions where you want to start after the initial for some reason? This is your fave thing
            steps:        List; of indices --Have a list of positions that only needs the 3rd and 5th steps? This thing is balling
            control_mode: INT;  -- defualts to pose. If your method_sub_map isn't a standard type you should probably change this.
            
            NOTE:
            if you're giving a position AND timeinterval, they MUST be the same length
            if you're only giving a position you can override the default timelength for the intervals w/ timelength
            trying to self.runback on an instance where reset==False *will* raise an exception
            
            Methods:
            run         : Runs all available positions in sequence
            runfwd      : Runs only forward steps
            runback     : Runs only backward steps (needs reset=True)
            execstep    : executes a particular step
            
            ### examples:
            # standard control
            CommandMessage(poslist)
            # joint control
            CommandMessage(poslist,method=None, method_sub_map=METHOD_SUB_MAP_JOINTS, control_mode=JOINT_VELOCITY)
        """
        self.controller = controller
        self.control_mode = control_mode
        if self.control_mode == 4:
            self.control_frame = "gripper"
            
        else:
            self.control_frame = "baseFrame"
        self.control_frame = None
            
            
        #       if we're interfacing stuff at the top level we'll input method = None
        self.method = method or "" 
        self.method_sub_map = method_sub_map
        if positions and timeinterval:
            if len(timeinterval) != len(positions):
                raise Exception("Length of Position and Timeinterval must be same")
            
            else:
                self.positions = positions
                self.timeinterval = timeinterval
        
        elif positions and not timeinterval:
            self.positions = positions
            self.timeinterval =  [timelength for i in self.positions]
            
        elif timeinterval and not positions:
            self.timeinterval = timeinterval
            self.positions = [(0,0,0,1) for i in self.timeinterval]
            
        else:
            raise Exception("Either a position or timeinterval set must be given.")
        
        self.poscnt = len(positions)
        ### 
        self.reset = reset
               
        # important later
        self.startingstep = startingstep
        self.currentstep = self.startingstep
        self.runcnt_fwd = 0
        self.runcnt_back = 0
        
        # can skip certain positions and repeat others. Ball Wildly
        if steps:
            for s in steps:
                if s > len(positions):
                    raise Exception("Step Index Values must be less than number of positions (duh)")
        
            else:
                tmp = []
                tmp.extend([self.positions[x] for x in steps])
                self.positions = tmp
        #
        if self.reset:
            self.initial = currentPose.current_position.position
            # if reversed, we need to reverse our positions and tack them onto themselves minus the last one.
            self.backpositions = list(reversed([negate(x) for x in self.positions]))
            self.backtimeintervals = list(reversed(self.timeinterval))
            
            self.fwdpositions = self.positions
            self.fwdtimeintervals = self.timeinterval
            
            self.positions = self.fwdpositions + self.backpositions
            self.timeinterval = self.fwdtimeintervals + self.backtimeintervals
        #
        self.done = False
            
    def run(self,ov_pos=None,ov_time=None):
        #ov_pos/time override pos/time
        positions = ov_pos or self.positions
            
        for i in range(self.currentstep,len(positions)):
            self.execstep(ov_pos=ov_pos,ov_time=ov_time)
            self.currentstep += 1
        self.currentstep = self.startingstep
        
    def runfwd(self):
        self.run(ov_pos=self.fwdpositions,ov_time=self.fwdtimeintervals)
        self.runcnt_fwd += 1
        
    def runback(self):
        if self.reset == True:
            self.run(ov_pos=self.backpositions,ov_time=self.backtimeintervals)
            self.runcnt_back +=1
        else:
            raise Exception("Can't just run back if not initialized w/ reset. What do you think we are?")
                
    #def execstep(self,step=None,ov_pos=None,ov_time=None):
        ##other is a tuple composed of (positions,timeintervals)
        #positions = ov_pos or self.positions
        #timeintervals = ov_time or self.timeinterval
        #thisstep = step or self.currentstep
        #try:
            #### TODO: IDEALLY here we actually do something with the arm. 
            #now = rospy.Time.now()

            #for i,j in zip(positions[thisstep],self.method_sub_map) :
               #rec_setattr(self.controller,self.method+j,i)
            ##print positions[thisstep],
            ##print thisstep,
            #print self.buildControlMsg()
            #rospy.sleep(timeintervals[thisstep])
            ##print self.controller
            #self.controller.publish(self.buildControlMsg())
        #except IndexError:
            #print "Bad Step, Doesn't Exist"
    def execstep(self,step=None,ov_pos=None,ov_time=None):
        positions = ov_pos or self.positions
        timeintervals = ov_time or self.timeinterval
        thisstep = step or self.currentstep
        print current_info
        print positions[thisstep]
        while list(current_info) != list(positions[thisstep]):
            print "moving"
            print current_info
            print positions[thisstep]
            for desired,method,current in zip(positions[thisstep],self.method_sub_map,current_info):
                minimult = (desired-current)/20
                #print "mm: " + str(minimult)
                if desired > current:
                    movement = 1 * MULT * (abs(minimult) +1 )
                elif desired < current:
                    movement = -1 * MULT * (abs(minimult) +1 )
                else:
                    movement = 0
                #print desired,method,current,movement
                rec_setattr(self.controller,self.method+method,movement)
            #print self.buildControlMsg()
            desiredPosePublisher.publish(self.buildControlMsg())
        print "reached"
            
    def should_reset(self):
        if self.runcnt_fwd > self.runcnt_back:
            return True
        else:
            return False
    def buildControlMsg(self):
        '''
        Takes in a RobolinkControl() message and an array of jointVelocities and builds a message
        '''
        robolinkControlMsg = self.controller
        #print dir(self.controller)
        #robolinkControlMsg = RobolinkControl()
        #print dir(robolinkControlMsg)
        
        #Build the header
        now = rospy.Time.now()
        if self.control_frame:
            robolinkControlMsg.header.frame_id = self.control_frame
        robolinkControlMsg.header.stamp = now
        
        #Set the control mode
        robolinkControlMsg.control_mode = self.control_mode
        
        return robolinkControlMsg
            
            
class CommandAcceptor(object):
    """ 
    gets initialized with a dictionary of command messages.
    
    ### example
    cmddict = {'fwd':CommandMessage(positions=[(0,1,2),(1,2,3)]), 'back':CommandMessage(positions=[(0,-1,-2),(-1,-2,-3)])}
    inst = CommandAcceptor(cmddict)
    inst.spin()
    #inst.run('fwd')
    #inst.reset()
    
    inst.run('back')
    inst.reset()
    
    TODO: add locks

    """
    def __init__(self,cmddict):
        self.commanddict = cmddict
        self.currentcmd = None
        
    def run(self,cmd):
        if self.commanddict.has_key(cmd):
            self.currentcmd = self.commanddict[cmd]
            if self.currentcmd.reset:
                self.currentcmd.runfwd()
            else:
                self.currentcmd.run()
        else:
            raise Exception("Invalid Command")

    def reset(self):
        if self.currentcmd.reset:
            self.currentcmd.runback()
        else:
            raise Exception("Can't run a reset cmd without a reset")
    def can_reset(self):
        if self.currentcmd.reset:
            return True
        else:
            return False
        
    def spin(self):
        while True:
            pass
### initialize the commandacceptor object
def initialize():
    
    global cmdacceptor
    cmddict = {
        #'none':CommandMessage(  positions=[(0.0,0,0),    (0.0,0,0)]), 
        #'fwd':CommandMessage(   positions=[(0.05,0,0),   (0.1,0,0)]), 
            #'fwd':CommandMessage(   positions=[(0.05,0,0),]), 
        #'back':CommandMessage(  positions=[(-0.05,0,0),  (-0.1,0,0)]),
        #'back':CommandMessage(  positions=[(-0.05,0,0)]),
        #'left':CommandMessage(  positions=[(0,.025,0),   (0,.5,0)]),
        #'right':CommandMessage( positions=[(0,-.025,0),  (0,-0.5,0)]),
        
        'reset': CommandMessage( positions = [(0,0,0,0,0),],method=None,reset=False,method_sub_map=METHOD_SUB_MAP_JOINTS,control_mode=JOINT_VELOCITY),
        #'five': CommandMessage( positions = [(0,5,0,5,0),],method=None,reset=False,method_sub_map=METHOD_SUB_MAP_JOINTS,control_mode=JOINT_VELOCITY),
        
        'drive_neut': CommandMessage( positions = [(4,0,0,80,1),(4,-2,0,82,1),],method=None,reset=False,method_sub_map=METHOD_SUB_MAP_JOINTS,control_mode=JOINT_VELOCITY),
        'drive_stop': CommandMessage( positions = [(4,-2,0,82,1)],method=None,reset=False,method_sub_map=METHOD_SUB_MAP_JOINTS,control_mode=JOINT_VELOCITY),
        'drive_fwd': CommandMessage( positions = [(4,-2,0,76,1)],method=None,reset=False,method_sub_map=METHOD_SUB_MAP_JOINTS,control_mode=JOINT_VELOCITY),
        'drive_back': CommandMessage( positions = [(4,-2,0,86,1)],method=None,reset=False,method_sub_map=METHOD_SUB_MAP_JOINTS,control_mode=JOINT_VELOCITY),(-24, 12, 3, 38, 0)
        
        #  HAHA DISREGARD THAT I CANT PUT 3Lbs of FORCE
        'light_down': CommandMessage( positions = [(-24, 21, 5, 31, 0), ],method=None,reset=False,method_sub_map=METHOD_SUB_MAP_JOINTS,control_mode=JOINT_VELOCITY),

        
        }
    cmdacceptor = CommandAcceptor(cmddict)
    return cmddict.keys()
    
def currentposCallback(RLInfo):
    global current_info
    current_info = RLInfo.joint_angles
def cmdCallback(string):
    print "Running Commmand: '%s'" % string.data
    cmdstr = string.data
    if cmdstr == "stop":        
        if cmdacceptor.can_reset() and cmdacceptor.currentcmd.should_reset():
            cmdacceptor.reset()
    else:
        if cmdacceptor.currentcmd:
            if cmdacceptor.currentcmd.should_reset():
                cmdacceptor.reset()
        
        cmdacceptor.run(string.data)
    #print cmdacceptor.currentcmd
    
if __name__ == '__main__':
    
    try:
        rospy.init_node('CommandAcceptor')
        keys = initialize()      
        rospy.Subscriber("Robolink_Info", RobolinkInfo, currentposCallback)
        rospy.Subscriber("cmdacceptor", String, cmdCallback)
        
        #rospy.Timer(rospy.Duration(0.1), publishDesiredPoseCallback)
        print "CommandAcceptor Initialized with commands: %s,stop" % ",".join(k for k in keys)
        rospy.spin()
    except rospy.ROSInterruptException:
        raise
    
    finally:
        pass
            #setAllVelocities(0,0,0,0,0)                 
            
            
# wheel drive model no- RL-ZA001-0250-12PFN