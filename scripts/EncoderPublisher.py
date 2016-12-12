#!/usr/bin/env python
# license removed for brevity
from Quickbot import Encoder
import rospy
from arm_quickbot.msg import EncodersMsg

def publishEncoders(enLeft,enRight):
    pub = rospy.Publisher('EncodersData', EncodersMsg, queue_size=1)
    rospy.init_node('EncodersPublisher')
    rate = rospy.Rate(20) # [Hz]
    msg = EncodersMsg()
    while not rospy.is_shutdown():
        enLeft.calcCountAndVelocity()
        enRight.calcCountAndVelocity()
        msg.encoderLeft = enLeft.count
        msg.encoderRight = enRight.count
        pub.publish(msg)
        rospy.loginfo(msg)
        rate.sleep()

if __name__ == '__main__':
    enLeft = Encoder("P8_14")
    enRight = Encoder("P8_15")
    enLeft.setInterrupt()
    enRight.setInterrupt()
    try:
        publishEncoders(enLeft,enRight)
    except rospy.ROSInterruptException:
        pass
