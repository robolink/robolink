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
METHOD_SUB_MAP_POS = ['position.x','position.y','position.z']
METHOD_SUB_MAP_OR = ['orientation.x','orientation.y','orientation.z','orientation.w']
METHOD_SUB_MAP = METHOD_SUB_MAP_POS + METHOD_SUB_MAP_OR

#super useful
def rec_getattr(obj, attr):
    return reduce(getattr, attr.split("."), obj)
def rec_setattr(obj, attr, value):
    attrs = attr.split(".")
    setattr(reduce(getattr, attrs[:-1], obj), attrs[-1], value)

class CommandMessage(object):
    def __init__(self, positions=[], timeinterval=[], timelength=0.1, reset=True, startingstep=0, steps=[], controller=desiredPose, method="pose.", method_sub_map =METHOD_SUB_MAP_POS):
        """ ARGS: 
        
            position:     List; of positions
            timeinterval: List; of times to idle between positions
            timelength:   Int;  Overrides the default timelength that is passed into the function when generating a timeinterval
            reset:        Bool; --go back to where when started (useful for driving, less for controlling light switches)
            startingstep: Int ; --Have a list of positions where you want to start after the initial for some reason? This is your fave thing
            steps:        List; of indices --Have a list of positions that only needs the 3rd and 5th steps? This thing is balling
            
            NOTE:
            if you're giving a position AND timeinterval, they MUST be the same length
            if you're only giving a position you can override the default timelength for the intervals w/ timelength
            trying to self.runback on an instance where reset==False *will* raise an exception
            
            Methods:
            run         : Runs all available positions in sequence
            runfwd      : Runs only forward steps
            runback     : Runs only backward steps (needs reset=True)
            execstep    : executes a particular step
        """
        self.controller = controller
        self.method = method
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
        self.fwdruncnt = 0
        self.backruncnt = 0
        
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
            self.backpositions = list(reversed(self.positions))
            self.backtimeintervals = list(reversed(self.timeinterval))
            
            self.fwdpositions = self.positions
            self.fwdtimeintervals = self.timeinterval
            
            self.positions = self.fwdpositions + self.backpositions[1:]
            self.timeinterval = self.fwdtimeintervals + self.backtimeintervals[1:]
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
        self.fwdruncnt += 1
        
    def runback(self):
        if self.reset == True:
            self.run(ov_pos=self.backpositions,ov_time=self.backtimeintervals)
            self.backruncnt +=1
        else:
            raise Exception("Can't just run back if not initialized w/ reset. What do you think we are?")
                
    def execstep(self,step=None,ov_pos=None,ov_time=None):
        #other is a tuple composed of (positions,timeintervals)
        positions = ov_pos or self.positions
        timeintervals = ov_time or self.timeinterval
        thisstep = step or self.currentstep
        try:
            ### TODO: IDEALLY here we actually do something with the arm. 
            for i,j in zip(positions[thisstep],self.method_sub_map) :
                rec_setattr(self.controller,self.method+j,i)
            print positions[thisstep],
            print thisstep,
            print timeintervals[thisstep]
        except IndexError:
            print "Bad Step, Doesn't Exist"
            
            
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
        'fwd':CommandMessage(positions=[(0,1,2),(1,2,3)]), 
        'back':CommandMessage(positions=[(0,-1,-2),(-1,-2,-3)])
        }
    cmdacceptor = CommandAcceptor(cmddict)
    return cmddict.keys()
    
def cmdCallback(string):
    print "Running Commmand: '%s'" % string.data
    if cmdacceptor.currentcmd:
        if cmdacceptor.can_reset():
            cmdacceptor.reset()
        
    cmdacceptor.run(string.data)
    #print cmdacceptor.currentcmd
    
if __name__ == '__main__':
    
    try:
        rospy.init_node('CommandAcceptor')
        keys = initialize()      
        rospy.Subscriber("cmdacceptor", String, cmdCallback)
        #rospy.Timer(rospy.Duration(0.1), publishDesiredPoseCallback)
        print "CommandAcceptor Initialized with commands: %s" % ",".join(k for k in keys)
        rospy.spin()
    except rospy.ROSInterruptException:
        raise
    
    finally:
        pass
        #setAllVelocities(0,0,0,0,0)