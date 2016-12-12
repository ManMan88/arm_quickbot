#!/usr/bin/env python
from Quickbot import Odometry
import rospy
from arm_quickbot.msg import EncodersMsg
from arm_quickbot.msg import LocationMsg

class Odometer(Odometry):
    def __init__(self):
        super(Odometer,self).__init__()
        self.pub = rospy.Publisher('Location', LocationMsg, queue_size=1)
        rospy.init_node('OdometryNode')
        self.pubMsg = LocationMsg()

    def _publishLocation(self,encoderData):
       self.calcLocation(encoderData.encoderRight,encoderData.encoderLeft)
       self.pubMsg.x = self.x
       self.pubMsg.y = self.y
       self.pubMsg.theta = self.theta
       self.pub.publish(self.pubMsg)
       rospy.loginfo("The Location is x = %.2f, y = %.2f, theta = %.2f",self.pubMsg.x,self.pubMsg.y,self.pubMsg.theta)
    def getEncoderDataPublishLocation(self):
        rospy.Subscriber("EncodersData", EncodersMsg, self._publishLocation)
        rospy.spin()

if __name__ == '__main__':
    odo = Odometer()
    odo.getEncoderDataPublishLocation()
