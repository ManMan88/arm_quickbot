#!/usr/bin/env python
# license removed for brevity
from Quickbot import Encoder
import rospy
from arm_quickbot.msg import EncodersMsg
import sys

def publishEncoders(enLeft,enRight):
    pub = rospy.Publisher('EncodersData', EncodersMsg, queue_size=1)
    rospy.init_node('EncodersPublisher')
    rate = rospy.Rate(20) # [Hz]
    msg = EncodersMsg()
    while not rospy.is_shutdown():
        enLeft.calcCountAndVelocity()
        enRight.calcCountAndVelocity()
        msg.encoderLeftCount = enLeft.count
        msg.encoderRightCount = enRight.count
        msg.encoderLeftRate = enLeft.velocity
        msg.encoderRightRate = enRight.velocity
        pub.publish(msg)
        rospy.loginfo(msg)
        rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) == 3:
    	pinLeft = sys.argv[1]
        pinRight = sys.argv[2]
    else:
        print("Worng number of arguments")
        sys.exit(1)    
    enLeft = Encoder(pinLeft)
    enRight = Encoder(pinRight)
    enLeft.setInterrupt()
    enRight.setInterrupt()
    try:
        publishEncoders(enLeft,enRight)
    except rospy.ROSInterruptException:
        pass
