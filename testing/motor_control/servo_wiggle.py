#!/usr/bin/python
# created by STCDEV001 & ANDNIC019, 01/09/2018

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(3, GPIO.OUT)
pwm = GPIO.PWM(3,50)
pwm.start(0)

def SetAngle(angle):
    duty = angle / 18 +2
    GPIO.output(3, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.3)
    GPIO.output(3, False)
    pwm.ChangeDutyCycle(0)

try:
    while True:
##        SetAngle(117+10)
##        time.sleep(0.2)
        SetAngle(65)
##        time.sleep(0.2)
##        SetAngle(40)
##        time.sleep(0.2)
##        SetAngle(30)
##        time.sleep(0.5)
finally:
    GPIO.cleanup()

