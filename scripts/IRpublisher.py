#!/usr/bin/env python
# license removed for brevity
from Quickbot import IRSensors
import rospy
from arm_quickbot.msg import IRsensorsMsg

def publishIR(IRlist):
    pub = rospy.Publisher('IRsensorsDistance', IRsensorsMsg, queue_size=1)
    rospy.init_node('IRpublisher')
    rate = rospy.Rate(5) # [Hz]
    n = len(IRlist)
    msg = IRsensorsMsg()
    while not rospy.is_shutdown():
	for ind in range(n):
	    msg.IRdata[ind] = IRlist[ind].irDist()
        rospy.loginfo(msg.IRdata)
        pub.publish(msg.IRdata)
        rate.sleep()

if __name__ == '__main__':
    irLB = IRSensors("P9_35","LB")
    irLF = IRSensors("P9_36","LF")
    irCF = IRSensors("P9_37","CF")
    irRF = IRSensors("P9_38","RF")
    irRB = IRSensors("P9_39","RB")
    IRlist = [irLB,irLF,irCF,irRF,irRB]
    try:
        publishIR(IRlist)
    except rospy.ROSInterruptException:
        pass
