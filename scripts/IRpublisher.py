[A#!/usr/bin/env python
# license removed for brevity
from Quickbot import IRSensors
import rospy
from arm_quickbot.msg import IRsensorsMsg
import sys

def publishIR(IRlist):
    pub = rospy.Publisher('IRsensorsDistance', IRsensorsMsg, queue_size=1)
    rospy.init_node('IRpublisher')
    rate = rospy.Rate(5) # [Hz]
    n = len(IRlist)
    msg = IRsensorsMsg()
    for ind in range(n):
        msg.IRangles[ind] = IRlist[ind].angle
    while not rospy.is_shutdown():
	for ind in range(n):
	    msg.IRvalues[ind] = IRlist[ind].irDist()
        rospy.loginfo(msg.IRvalues)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) == 6:
        pinLB = sys.argv[1]
	pinLF = sys.argv[2]
	pinCF = sys.argv[3]
	pinRF = sys.argv[4]
	pinRB = sys.argv[5]
    else:
        print("Wrong number of input arguments")
        sys.exit(1)
    irLB = IRSensors(pinLB,"LB",-90)
    irLF = IRSensors(pinLF,"LF",-45)
    irCF = IRSensors(pinCF,"CF",0)
    irRF = IRSensors(pinRF,"RF",45)
    irRB = IRSensors(pinRB,"RB",90)
    IRlist = [irLB,irLF,irCF,irRF,irRB]
    try:
        publishIR(IRlist)
    except rospy.ROSInterruptException:
        pass
