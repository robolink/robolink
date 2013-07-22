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

class CommandMessage(object):
    def __init__(self, positions=[], timeinterval=[], timelength=0.1, reset=True, startingstep=0, steps=[]):
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
        
    def runback(self):
        if self.reset == True:
            self.run(ov_pos=self.backpositions,ov_time=self.backtimeintervals)
        else:
            raise Exception("Can't just run back if not initialized w/ reset. What do you think we are?")
                
    def execstep(self,step=None,ov_pos=None,ov_time=None):
        #other is a tuple composed of (positions,timeintervals)
        positions = ov_pos or self.positions
        timeintervals = ov_time or self.timeinterval
        thisstep = step or self.currentstep
        try:
            ### TODO: IDEALLY here we actually do something with the arm. 
            print positions[thisstep],
            print thisstep,
            print timeintervals[thisstep]
        except IndexError:
            print "Bad Step, Doesn't Exist"