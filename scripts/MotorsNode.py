#!/usr/bin/env python
#license removed for brevity

from Quickbot import Motor
from numpy import sign
from math import pi
import rospy
from std_msgs.msg import Float64
from arm_quickbot.msg import ControlParamMsg
from arm_quickbot.srv import SetDirectionSrv
import sys

class MotorsNode(object):
    def __init__(self,leftMotor,rightMotor,diameter=63.5,baseLength=50, ticks=16):
	self.rightMotor = rightMotor
        self.leftMotor = leftMotor
        self.diameter = diameter
        self.baseLength = baseLength
        self.velCmd = 0
        self.omegaCmd = 0
        self.velR = 0
        self.velL = 0
        self.ticks = ticks
        self.pubLeftMotorSetpoint = rospy.Publisher('LeftMotorSetpoint', Float64, queue_size=1)
        self.pubRightMotorSetpoint = rospy.Publisher('RightMotorSetpoint', Float64, queue_size=1)
        rospy.init_node('MotorsNode')
        self.leftEnDirectionSrv = rospy.ServiceProxy('set_left_encoder_direction', SetDirectionSrv)
        self.rightEnDirectionSrv = rospy.ServiceProxy('set_right_encoder_direction', SetDirectionSrv)
        self.leftMotorSetpoint = Float64()
        self.rightMotorSetpoint = Float64()

    def _controlParam(self,parameters):
        self.velCmd = parameters.velocity

    def _omegaParam(self,param):
        self.omega.des = param.data

    def _uni2diff(self):
        # [velCmd] = [mm/sec],  [omegaCmd] = [rad/sec] , [velL] = [velR] = [rad/sec}
        self.velL = (2*self.velCmd - self.omegaCmd*self.baseLength)/self.diameter
        self.velR = (2*self.velCmd + self.omegaCmd*self.baseLength)/self.diameter

    def _ensureOmega(self):
        maxV = max(abs(self.velR),abs(self.velL))
        minV = min(abs(self.velR),abs(self.velL))
        signR = sign(self.velR)
        signL = sign(self.velL)
        if maxV > self.rightMotor.maxVel:
            self.velR = self.velR - signR*maxV + signR*self.rightMotor.maxVel
            self.velL = self.velL - signL*maxV + signL*self.leftMotor.maxVel
            minV = min(abs(self.velR),abs(self.velL))
        if minV < self.rightMotor.minVel:
                if abs(self.velR) < abs(self.velL):
                    self.velR = signR*self.rightMotor.minVel
                else:
                    self.velL = signL*self.leftMotor.minVel
	elif minV < rightMotor.minVel:
            self.velR = self.velR - signR*minV + signR*self.rightMotor.minVel
            self.velL = self.velL - signL*minV + signL*self.leftMotor.minVel
            maxV = max(abs(self.velR),abs(self.velL))
            if maxV > self.rightMotor.maxVel:
                if abs(self.velR) > abs(self.velL):
                    self.velR = signR*self.rightMotor.maxVel
                else:
                    self.velL = signL*self.leftMotor.maxVel

    def _setLeftMotor(self,effortInput):
        if not (sign(self.velL) == self.leftMotor.direction):
            self.leftEnDirectionSrv()
            leftMotor.reverseDirection()
        # transform to [counts/sec]
        self.velL = self.velL*self.ticks/(2*pi)
        self.leftMotor.inputPWM(effortInput.data)

    def _setRightMotor(self,effortInput):
        if not (sign(self.velR) == self.rightMotor.direction):
            self.leftEnDirectionSrv()
            leftMotor.reverseDirection()
        # transform to [counts/sec]
        self.velR = self.velR*self.ticks/(2*pi)
        self.rightMotor.inputPWM(effortInput.data)

    def runMotors(self):
        rospy.Subscriber("ControlParameters", ControlParamMsg, self._controlParam)
        rospy.Subscriber("OmegaEffort", Float64, self._omegaParam)
        rospy.Subscriber("leftMotorInput", Float64, self._setLeftMotor)
        rospy.Subscriber("rightMotorInput", Float64, self._setRightMotor)
        rate = rospy.Rate(40) #[Hz]
        while not rospy.is_shutdown():
            self._uni2diff()
            self._ensureOmega()
            self.leftMotorSetpoint.data = self.velL
            self.leftMotorSetpoint.data = self.velR
            self.pubLeftMotorSetpoint.publish(self.leftMotorSetpoint)
            self.pubRightMotorSetpoint.publish(self.rightMotorSetpoint)
            rospy.loginfo(self.leftMotorSetpoint)
            rospy.loginfo(self.rightMotorSetpoint)
            rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) == 7:
        pinEnL = sys.argv[1]
        pin1L = sys.argv[2]
        pin2L = sys.argv[3]
        pinEnR = sys.argv[4]
        pin1R = sys.argv[5]
        pin2R = sys.argv[6]
    else:
        print("Worng number of arguments")
        sys.exit(1)
    rospy.wait_for_service('set_left_encoder_direction', timeout = 10)
    rospy.wait_for_service('set_right_encoder_direction', timeout = 10)
    leftMotor = Motor(pinEnL,pin1L,pin2L)
    rightMotor = Motor(pinEnR,pin1R,pin2R)
    moto = MotorsNode(leftMotor,rightMotor)
    try:
        moto.runMotors()
    except rospy.ROSInterruptException:
        pass
