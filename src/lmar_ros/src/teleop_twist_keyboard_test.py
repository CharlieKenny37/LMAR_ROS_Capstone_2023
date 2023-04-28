#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import numpy as np
from geometry_msgs.msg import Twist

class motor_gpio():
    
    
    def __init__(self):
        rospy.init_node("motor_controller")
        rospy.Subscriber('cmd_vel', Twist ,self.motor_callback)
        
        while(not rospy.is_shutdown()):
            ""

        GPIO.cleanup()
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
        rospy.logerr("LINEAR X:")
        rospy.logerr(msg.linear.x)
        rospy.logerr("ANGULAR Z:")
        rospy.logerr(msg.angular.z)




        

if __name__ == "__main__":
    motor_gpio()
