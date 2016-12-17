import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import numpy as np
from math import cos, sin
import time

class IRSensors(object):
    def __init__(self,pin,pose):
        #Recieves a list of angles and coresponding input ADC pins
        #Create a list of sensors
        self.pose = pose
        self.pin = pin
        #how can I define voltages and distances only once and not to each instance? Global?
        #I need ro define goloba vairables in ROS -> meantime I stay like this
        self.voltages = np.array([0.357,0.42,0.519,0.654,0.731,0.813,0.927,1.061,
                                  1.267,1.397,1.562,1.763,2.017,2.335,2.72,2.994])
        self.distances = np.array([[35,30,25,20,18,16,14,12,10,9,8,7,6,5,4,3.5]]) 
        ADC.setup()
            
    def _readRawValue(self):
        return ADC.read_raw(self.pin)
        
    def irDist(self):
        return np.interp(self._readRawValue(),self.voltages,self.distances)
    
    def terminate(self):
        ADC.cleanup()

class Encoder(object):
    def __init__(self, pin):
        self.pin = pin
        self.totalCount = 0
        self.lastCount = 0
        self.direction = 1
        self.lastTime = time.clock()
        self.timeNew = time.clock()
        self.count = 0
        self.velocity = 0
        self.dt = 1
        GPIO.setup(pin, GPIO.IN)
    
    def _callbackCount(self):
        self.timeNew = time.clock()
        self.totalCount = self.totalCount + self.direction
    
    def reverseDirection(self, direction):       
        self.direction = -self.direction
        
    def setInterrupt(self):
        GPIO.add_event_detect(self.pin, GPIO.RISING, callback=self._callbackCount, bouncetime=100)
        
    def _shutdownInterrupt(self):
        GPIO.remove_event_detect(self.pin)
    
    def calcCountAndVelocity(self):
        temp = [self.totalCount,self.timeNew]
        self.count = (temp[0] - self.lastCount)     #[counts]
        self.dt = temp[1]-self.lastTime             #[msec]
        if self.dt > 0.00001:
            self.velocity = self.count/self.dt          #[counts/msec]
        else:
            self.velocity = 0
        self.lastTime = temp[1]        
        self.lastCount = temp[0]
    
    def terminate(self):
        self._shutdownInterrupt()
        GPIO.cleanup()

class Motor(object):
    def __init__(self,pinEn,pin1,pin2,maxVel=150):
        self.pineEn = pinEn
        self.pin1 = pin1
        self.pin2 = pin2
        self.targetVelocity = 0
        self.direction = 1
        self.maxVel = maxVel #[mm/sec] -> for both directions
        PWM.start(pinEn,0)
        GPIO.setup(pin1,GPIO.OUT)
        GPIO.setup(pin2,GPIO.OUT)
        GPIO.output(pin1,GPIO.HIGH)
        GPIO.output(pin2,GPIO.LOW)
        
    def _reverseDirection(self):
        self.direction = - self.direction
        if self.direction > 0:
            GPIO.output(self.pin1,GPIO.HIGH)
            GPIO.output(self.pin2,GPIO.LOW)
        else:
            GPIO.output(self.pin1,GPIO.LOW)
            GPIO.output(self.pin2,GPIO.HIGH)
    
    def setTargetVelocity(self,desiredVelocity):
        #the desiredVelocity should be between [-maxVel,maxVel]
        desiredDirection = p.sign(desiredVelocity)     
        if abs(desiredVelocity) > maxVel:
            desiredVelocity = maxVel*desiredDirection
        if desiredDirection == - self.direction:
            self._reverseDirection()
        self.targetVelocity = desiredVelocity
    
    def inputPWM(self, value):
        #the value should be between 0 and 100
        if value < 0:
            value = 0
        if value > 100:
            value = 100
        PWM.set_duty_cycle(self.pinEn,value)
        
    def terminate(self):
        PWM.stop(self.pinEn)
        PWM.cleanup()
        self.encoder.terminate()
        GPIO.cleanup()

class Odometry(object):
    def __init__(self, ticks=16, perimeter=199.5, baseLength=10, x0 = 0, y0 = 0, theta0 = 0):
        self.resolution = perimeter/ticks
        self.x = x0
        self.y = y0
        self.theta = theta0

    def calcLocation(self,rightWheelDist,leftWheelDist):
        distance = (rightWheelDist + leftWheelDist)*self.resolution/2
        self.x = self.x + distance*cos(self.theta)
        self.y = self.y + distance*sin(self.theta)
        self.theta = (rightWheelDist - leftWheelDist)*self.resolution/2
        #also possible to solve with ODE on the velocity but I find it wastefull

class Planner(object):
    def __init__(self):
        self.IRdata = [0,0,0,0,0]
        self.location = [0,0,0]
        self.target = "Stop"
        self.velCmd = 0
        self.thetaCmd = 0

    def states(self):
        if state == "Stop":
            self._stop()
        elif state == "AvoidObstacles":
            self._avoidObstacles()
        elif state == "FollowWall":
            self._followWall()
        elif state == "GoToGoal":
            self._goToTarget()
        else:
            self._stop()
        return 0

    def _followWall(self):
        return 0
    def _goToTarget(self):
        return 0
    def _avoidObstacles(self):
        return 0
    def _stop(self):
        return 0

class StateMachine(object):
    def __init(self, stopError=20, emergancyDist=4, avoidObstaclesDist=10, followWallDist=22, wallBias=30, factorIR=[1,5,10,5,1], target=[0,0]):
        self.state = "Stop"
        self.factorIR = factorIR
        self.stopError = stopError #[mm]
        self.targt = target
        self.followWall = 0
        self.distanceFromTargert = 0
        self.distanceStartWall = 0
        self.location = [0,0,0]
        self.IRdata = [0,0,0,0,0]
        self.minIRdata = 0
        self.avoidDist = avoidDist
        self.followWallDist = followWallDist
        self.emergancyDist = emergancyDist
        self.wallBias = wallBias
        
    def setTarget(self,target):
        self.target = target
        self.distanceFromTargert = np.linalg.norm(self.location[0:1] - self.target)
        return 0

    def _checkEmergency(self):
        if self.minIRdata < self.emergancyDist:
            return 1
        return 0

    def _checkReachedTarget(self):
        if self.distanceFromTargert < self.stopError:
            return 1
        return 0
    
    def _checkAvoidObstacles(self):
        if self.minIRdata < self.avoidObstaclesDist:
            return 1
        return 0

    def __checkFollowWall(self):
        if self.minIRdata < self.followWallDist:
            self.distanceStartWall = self.distanceFromTargert
            return 1
        return 0

    def _checkStopFollowWall(self):
        if self.distanceFromTargert + self.wallBias < self.distanceStartWall:
            return 1
        
        return 0



    
    def chooseState(self):
        self.distanceFromTargert = np.linalg.norm(self.location[0:1] - self.target)
        self.minIRdata = min(IRdata)
        if self._checkEmergency(IRdata):
            self.state = "Stop"
        elif self._checkReachedTarget(location):
            self.state = "Stop"
        elif self._checkAvoidObstacles(IRdata):
            self.state = "AvoidObstacles"
        elif self.followWall:
            if self._checkStopFollowWall:
                self.state = "GoToGoal"
                self.followWall = 0
        elif self._checkFollowWall(IRdata):
            self.state = "FollowWall"
            self.followWall = 1
        else:
            self.state = "GoToTarget"
            
        return self.state