#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import numpy as np

def leak_detector():
    
    Cylinders = ["Battery 1", "Batter 2", "Motor 1", "Motor 2", "Computer"]
    water_pins = np.array([13, 15, 16, 18, 11])
    GPIO.setmode(GPIO.BOARD)
    for x in np.arange(5):
        GPIO.setup(water_pins[x], GPIO.IN)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        for x in np.arange(5):
            if(GPIO.input(water_pins[x]) == 1):
                msg = ("Water Detected in " + Cylinders[x] + " Cylinder")
                rospy.logerr(msg)

        rate.sleep()
        

if __name__== "__main__":
    leak_detector()

