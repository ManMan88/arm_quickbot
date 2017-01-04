#!/usr/bin/env python

from Quickbot import Planner
import rospy
from arm_quickbot.srv import SetStateSrv
from arm_quickbot.msg import IRsensorsMsg, TargetMsg, LocationMsg, ControlParamMsg, VectorsMsg
from std_msgs.msg import Float64

class PlannerNode(Planner):
    def __init__(self):
        super(PlannerNode,self).__init__()
        self.pubControlParam = rospy.Publisher('ControlParameters', ControlParamMsg, queue_size = 1)
	self.pubRobotVectors = rospy.Publisher('RobotVectors', VectorsMsg, queue_size = 1)
	self.pubDesiredTheta = rospy.Publisher('DesiredTheta', Float64, queue_size = 1)
        rospy.init_node('PlannerNode')
        s = rospy.Service('set_state', SetStateSrv, self.states)
        self.cpMsg = ControlParamMsg()
	self.vecMsg = VectorsMsg()
	self.thetaMsg = Float64

    def _IRsensorsSub(self,IRdata):
        self.IRdata = IRdata

    def _locationSub(self,locationData):
        self.location = [locationData.x,locationData.y,locationData.theta]

    def _targetSub(self,targetData):
        self.target = targetData

    def runPlanner(self):
        rospy.Subscriber("Location", LocationMsg, self._locationSub)
        rospy.Subscriber("IRsensorsDistance", IRsensorsMsg, self._IRsensorsSub)
        rospy.Subscriber("Target", TargetMsg, self._targetSub)
        rate = rospy.Rate(10) #[Hz]
        while not rospy.is_shutdown():
            self.cpMsg.velocity = self.velDes
            self.cpMsg.theta = self.thetaDes
	    self.thetaMsg.data = self.cpMsg.theta
	    self.vecMsg.targetVector = self.targetVector
	    self.vecMsg.wallVector = self.wallVector
	    self.vecMsg.awayFromWallVector = self.awayFromWallVector
            self.pubControlParam.publish(self.cpMsg)
	    self.pubRobotVectors.publish(self.vecMsg)
	    self.pubDesiredTheta.publish(self.thetaMsg)
            rospy.loginfo(self.cpMsg)
	    rospy.loginfo(self.vecMsg)
            rate.sleep()

if __name__ == "__main__":
    planner = PlannerNode()
    try:
        planner.runPlanner()
    except rospy.ROSInterruptException:
        pass

