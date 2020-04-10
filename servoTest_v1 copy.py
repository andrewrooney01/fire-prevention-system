import RPi.GPIO as GPIO     # Import GPIO Library
import time                 # Import time library for a delay
from time import sleep
import numpy as np


thetaServoPin = 8 #blue data
phiServoPin = 10 #yellow data
GPIO.setmode(GPIO.BOARD) #set the naming mode to board mode

GPIO.setup(thetaServoPin, GPIO.OUT)
GPIO.setup(phiServoPin, GPIO.OUT)

theta = GPIO.PWM(thetaServoPin, 50)
phi = GPIO.PWM(phiServoPin, 50)
theta.start(0)
phi.start(0)

def TwoServo():
    x = 0
    while(x < 5):
        for i in range(45,135):
            positionl = 1./18.*(180-i)+1
            positionr = 1./18.*(180-i)+1
            theta.ChangeDutyCycle(positionl)
            phi.ChangeDutyCycle(positionr)
            time.sleep(0.005)
        
        for i in range(135,45,-1):
            positionl = 1./18.*(i)+2
            positionr = 1./18.*(180-i)+2
            theta.ChangeDutyCycle(positionl)
            phi.ChangeDutyCycle(positionr)
            time.sleep(0.005)
        x = x + 1
    theta.stop()
    phi.stop()
    
def SetThetaAngle(ang):
    duty = ang
    #angle = ang * 5
    #duty = angle / 18 + 1
    GPIO.output(thetaServoPin, True)
    theta.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(thetaServoPin, False)
    theta.ChangeDutyCycle(0)
    theta.stop()

def SetPhiAngle(ang):
    duty = ang
    #angle = ang * 2/0.75
    #duty = angle / 18 + 1
    GPIO.output(phiServoPin, True)
    phi.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(phiServoPin, False)
    phi.ChangeDutyCycle(0)
    phi.stop()

#def GoHome():
#phi_radians = phi*(2*np.pi/360)
#theta_radians = theta*(2*np.pi/360)

#TwoServo()
SetThetaAngle(50)
SetPhiAngle(30)
GPIO.cleanup()
