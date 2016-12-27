#!/usr/bin/env python

from Quickbot import StateMachine
import rospy
from arm_quickbot.srv import SetTargetSrv, SetStateSrv
from arm_quickbot.msg import IRsensorsMsg, StateMsg, TargetMsg, LocationMsg

class StateMachineNode(StateMachine):
    def __init__(self):
	super(StateMachineNode,self).__init__()
        self.pubState = rospy.Publisher('State', StateMsg, queue_size = 5)
	self.pubTarget = rospy.Publisher('Target', TargetMsg, queue_size = 5) 
        rospy.init_node('StateMachineNode')
        self.stateMsg = StateMsg()
	self.targetMsg = TargetMsg()
	self.s = rospy.Service('set_target', SetTargetSrv, self.setTarget)
	self.stateSrv = rospy.ServiceProxy('set_state', SetStateSrv)
    
    def _determineState(self):
        self.stateMsg.state = self.chooseState()

    def _IRsensorsSub(self,IRdata):
	self.IRvalues = IRdata.values

    def _locationSub(self,locationData):
        self.location = [locationData.x,locationData.y,locationData.theta]

    def runStateMashine(self):
	rospy.Subscriber("Location", LocationMsg, self._locationSub)
	rospy.Subscriber("IRsensorsDistance", IRsensorsMsg, self._IRsensorsSub)
	rate = rospy.Rate(10) #[Hz]
	while not rospy.is_shutdown():
	    self.stateMsg.state = self.chooseState()
	    if self.stateSrv(self.state):
		print "stateSrv error in StateMachineNode"
		raise 
            self.targetMsg.target = self.target
	    self.pubState.publish(self.stateMsg)
	    rospy.loginfo(stateMsg)
	    self.pubTarget.publish(self.targetMsg)
	    rospy.loginfo(targetMsg)
	    rate.sleep()

if __name__ == "__main__":
    rospy.wait_for_service('SetState', timeout = 10)
    SM = StateMachineNode()
    try:
        SM.runStateMachine()
    except rospy.ROSInterruptException:
        pass

