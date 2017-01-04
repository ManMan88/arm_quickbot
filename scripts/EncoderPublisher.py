#!/usr/bin/env python
# license removed for brevity
from Quickbot import Encoder
import rospy
from arm_quickbot.msg import EncodersMsg
from std_msgs.msg import Float64
from arm_quickbot.srv import SetDirectionSrv
import sys

def publishEncoders(enLeft,enRight):
    pub = rospy.Publisher('EncodersData', EncodersMsg, queue_size=1)
    pubRightOmega = rospy.Publisher('RightOmegaState', Float64, queue_size=1)
    pubLeftOmega = rospy.Publisher('LeftOmegaState', Float64, queue_size=1)
    rospy.init_node('EncodersPublisher')
    s = rospy.Service('set_left_encoder_direction', SetDirectionSrv, enLeft.reverseDirection)
    s = rospy.Service('set_right_encoder_direction', SetDirectionSrv, enRight.reverseDirection)
    rate = rospy.Rate(20) # [Hz]
    msg = EncodersMsg()
    rightOmeagaMsg = Float64()
    leftOmeagaMsg = Float64()
    while not rospy.is_shutdown():
        enLeft.calcCountAndVelocity()
        enRight.calcCountAndVelocity()
        msg.encoderLeftCount = enLeft.count
        msg.encoderRightCount = enRight.count
        msg.encoderLeftRate = enLeft.velocity
        msg.encoderRightRate = enRight.velocity
	leftOmegaMsg.data = msg.encoderLeftRate
	rightOmegaMsg.data = msg.encoderRightRate
        pub.publish(msg)
	pubLeftOmega.publish(leftOmegaMsg)
	pubRightOmega.publish(rightOmegaMsg)
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
