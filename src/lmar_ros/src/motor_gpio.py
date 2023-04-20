#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import numpy as np
from geometry_msgs.msg import TwistStamped

class motor_gpio():
    
    
    def __init__(self):
    
        rospy.init_node("motor_controller")
        self.last_time = rospy.time()
        self.pins = [32, 33]
        self.dir_pins = []
        GPIO.setmode(GPIO.BOARD)
        for x in self.pins:
            GPIO.setup(x, GPIO.OUT)
        rospy.Subscriber('cmd_vel', TwistStamped ,self.motor_callback)

        while(not rospy.is_shutdown):
            if(rospy.time() - self.last_time < 1):
                for x in np.size(self.pins):
                    GPIO.output(self.pins[x], self.last_msg)
                    GPIO.outpit(self.dir_pins[x], 1)
            else:
                for x in self.pins:
                    GPIO.output(self.pins[x], 0)
                    GPIO.outpit(self.dir_pins[x], 0)


        """
        while(not rospy.is_shutdown):
            if(rospy.time() - self.last_time < 1):
                for x in np.size(self.pins):
                    if(self.last_msg.twist.linear.x > 0.1:
                        GPIO.output(self.pins[x], self.last_msg)
                        GPIO.outpit(self.dir_pins[x], 1)
                    elif(np.abs(self.last_msg.twist.angular.z) > 0.1):
                        GPIO.output(self.pins[x], self.last_msg)
                        GPIO.outpit(self.dir_pins[x], x)
            else:
                for x in self.pins:
                    GPIO.output(self.pins[x], 0)
                    GPIO.outpit(self.dir_pins[x], 0)
        """



    def motor_callback(self, msg):
        self.last_time = rospy.time()
        self.last_msg = 255



        

if __name__== "__main__":
    motor_gpio()