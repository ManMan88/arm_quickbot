#!/usr/bin/env python
from Quickbot import Odometry
import rospy
from arm_quickbot.msg import EncodersMsg, LocationMsg
from std_msgs.msg import Float64

class Odometer(Odometry):
    def __init__(self):
        super(Odometer,self).__init__()
        self.pub = rospy.Publisher('Location', LocationMsg, queue_size=1)
	self.pubThetaState = rospy.Publisher('ThetaState', Float64, queue_size=1)
        rospy.init_node('OdometryNode')
        self.pubMsg = LocationMsg()
	self.pubTheta = Float64()

    def _publishLocation(self,encoderData):
        self.calcLocation(encoderData.encoderRightCount,encoderData.encoderLeftCount)
        self.pubMsg.x = self.x
        self.pubMsg.y = self.y
        self.pubMsg.theta = self.theta
	self.pubTheta.data = self.theta
        self.pub.publish(self.pubMsg)
	self.pubTheta.publish(self.pubTheta)
        rospy.loginfo("The Location is x = %.2f, y = %.2f, theta = %.2f",self.pubMsg.x,self.pubMsg.y,self.pubMsg.theta)

    def getEncoderDataPublishLocation(self):
        rospy.Subscriber("EncodersData", EncodersMsg, self._publishLocation)
        rospy.spin()

if __name__ == '__main__':
    odo = Odometer()
    try:
        odo.getEncoderDataPublishLocation() 
    except rospy.ROSInterruptException:
        pass
