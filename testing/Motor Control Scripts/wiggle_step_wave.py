from time import sleep
import RPi.GPIO as GPIO
import os
import math
#from random import randint
import random

GPIO.setmode(GPIO.BCM)

# Verwendete Pins des ULN2003A auf die Pins des Rapberry Pi
# zugeordnet 
IN1=6 # IN1
IN2=13 # IN2
IN3=19 # IN3
IN4=26 # IN4

# Wartezeit regelt die Geschwindigkeit wie schnell sich der Motor
# dreht.
time = 0.001

# Pins aus AusgÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂ¤nge definieren
GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)
GPIO.setup(IN3,GPIO.OUT)
GPIO.setup(IN4,GPIO.OUT)
# Alle Pins werden initial auf False gesetzt. So dreht sich der 
# Stepper-Motor nicht sofort irgendwie.
GPIO.output(IN1, False)
GPIO.output(IN2, False)
GPIO.output(IN3, False)
GPIO.output(IN4, False)

# Der Schrittmotoren 28BYJ-48 ist so aufgebaut, das der Motor im
# Inneren 8 Schritte fÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂ¼r eine Umdrehung benÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂ¶tigt. Durch die Betriebe
# benÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂ¤tigt es aber 512 x 8 Schritte damit die Achse sich einmal um
# sich selbt also 360ÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂ° dreht.

# Definition der Schritte 1 - 8 ÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂÃÂ¼ber die Pins IN1 bis IN4
# Zwischen jeder Bewegung des Motors wird kurz gewartet damit der
# Motoranker seine Position erreicht.
def Step1():
    GPIO.output(IN4, True)
    sleep (time)
    GPIO.output(IN4, False)

def Step2():
    GPIO.output(IN4, True)
    GPIO.output(IN3, True)
    sleep (time)
    GPIO.output(IN4, False)
    GPIO.output(IN3, False)

def Step3():
    GPIO.output(IN3, True)
    sleep (time)
    GPIO.output(IN3, False)

def Step4():
    GPIO.output(IN2, True)
    GPIO.output(IN3, True)
    sleep (time)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)

def Step5():
    GPIO.output(IN2, True)
    sleep (time)
    GPIO.output(IN2, False)

def Step6():
    GPIO.output(IN1, True)
    GPIO.output(IN2, True)
    sleep (time)
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)

def Step7():
    GPIO.output(IN1, True)
    sleep (time)
    GPIO.output(IN1, False)

def Step8():
    GPIO.output(IN4, True)
    GPIO.output(IN1, True)
    sleep (time)
    GPIO.output(IN4, False)
    GPIO.output(IN1, False)

# Umdrehung links herum  
def down(step):	
	for i in range (step):   
		#os.system('clear') # verlangsamt die Bewegung des Motors zu sehr. 
		Step1()
		Step2()
		Step3()
		Step4()
		Step5()
		Step6()
		Step7()
		Step8()  
		print("Step down: ",i)

# Umdrehung rechts herum		
def up(step):
	for i in range (step):
		#os.system('clear') # verlangsamt die Bewegung des Motors zu sehr.  
		Step8()
		Step7()
		Step6()
		Step5()
		Step4()
		Step3()
		Step2()
		Step1()  
		print( "Step up: ",i	)

step_angle = 360/520;
step = 0;
out_now = 0;
out_should_be = 0;
prev_out = 0;

freq = 0.05;
w = 2 * math.pi * freq;

amplitude = 20; #in degs

sleeptime =0.01;

for t in range(1,5):
    up(8);
    sleep(sleeptime);
    up(5)
    sleep(sleeptime);
    up(2)
    sleep(sleeptime);
    up(1)
    sleep(sleeptime);
    sleep(sleeptime);
    
    down(1);
    sleep(sleeptime);
    down(2);
    sleep(sleeptime);
    down(5)
    sleep(sleeptime);
    down(8)
    sleep(sleeptime);
    
    down(8);
    sleep(sleeptime);
    down(5)
    sleep(sleeptime);
    down(2)
    sleep(sleeptime);
    down(1)
    sleep(sleeptime);
    sleep(sleeptime);
    
    up(1);
    sleep(sleeptime)
    up(2);
    sleep(sleeptime);
    up(5)
    sleep(sleeptime);
    up(8)
    sleep(sleeptime);
    
##for t in range(1,100) :
##    
##    out_should_be = amplitude * math.sin(w*t)
##    
##    if (out_should_be > prev_out) :
##        up(round((out_should_be-prev_out) / step_angle));
##        out_now += round(out_should_be-prev_out);
##    elif out_should_be < prev_out :
##        down(round((prev_out-out_should_be) / step_angle));
##        out_now -= round(prev_out-out_should_be);
            
##    if (out_should_be > prev_out) :
##        while (out_now < out_should_be) :
##            up(1);
##            out_now += step_angle;
##    elif out_should_be < prev_out :
##        while (out_now > out_should_be) :
##            down(1);
##            out_now -= step_angle;
    
    prev_out = out_now;
    sleep(0.01);
