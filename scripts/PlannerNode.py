#!/usr/bin/env python

from Quickbot import Planner
import rospy
from arm_quickbot.srv import SetStateSrv
from arm_quickbot.msg import IRsensorsMsg, TargetMsg, LocationMsg, ControlParamMsg

class PlannerNode(Planner):
    def __init__(self):
        super(PlannerNode,self).__init__()
        self.pubControlParam = rospy.Publisher('ControlParameters', ControlParamMsg, queue_size = 1)
        rospy.init_node('PlannerNode')
        s = rospy.Service('set_state', SetStateSrv, self.states)
        self.cpMsg = ControlParamMsg()

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
            self.pubControlParam.publish(self.cpMsg)
            rospy.loginfo(self.cpMsg)
            rate.sleep()

if __name__ == "__main__":
    planner = PlannerNode()
    try:
        planner.runPlanner()
    except rospy.ROSInterruptException:
        pass

