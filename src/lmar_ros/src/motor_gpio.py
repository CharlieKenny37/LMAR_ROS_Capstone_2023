#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import numpy as np
from geometry_msgs.msg import Twist

class motor_gpio():
    
    
    def __init__(self):
        rospy.init_node("motor_controller")
        GPIO.setmode(GPIO.BOARD)

        #RIGHT SIDE
        self.pwm_right_pin = 32
        self.right_dir_pin = 36
        GPIO.setup(self.pwm_right_pin, GPIO.OUT)
        GPIO.setup(self.right_dir_pin, GPIO.OUT)
        self.pwm_right = GPIO.PWM(self.pwm_right_pin,4000)
        self.pwm_right.start(0)
        GPIO.output(self.right_dir_pin,0)

        
        #LEFT SIDE
        self.pwm_left_pin = 33
        self.left_dir_pin = 31
        GPIO.setup(self.pwm_left_pin, GPIO.OUT)
        GPIO.setup(self.left_dir_pin, GPIO.OUT)
        self.pwm_left = GPIO.PWM(self.pwm_left_pin,4000)
        self.pwm_left.start(0)
        GPIO.output(self.left_dir_pin,0)


        #GPIO.output(36,1)
        rospy.Subscriber('cmd_vel', Twist ,self.motor_callback)
        
        while(not rospy.is_shutdown()):
            pass
        GPIO.cleanup()


    def motor_callback(self, msg):

        #clips the speed if it is over 1.0, [0.0, 1.0] is the range, as a percentage of maximum speed
        if(msg.linear.x > 1.0):
            msg.linear.x = 1.0

        elif(msg.linear.x < -1.0):
            msg.linear.x = -1.0
        
        elif(msg.angular.z > 1.0):
            msg.angular.z = 1.0
        
        elif(msg.angular.z < -1.0):
            msg.angular.z = -1.0


        #determines the value in the message to send to each pwm pi, and what value to send to each direction pin
        if(msg.linear.x > 0.0):
            #right side
            self.pwm_right.ChangeDutyCycle(100*np.abs(msg.linear.x))
            GPIO.output(self.right_dir_pin, 1)

            #left side
            self.pwm_left.ChangeDutyCycle(100*np.abs(msg.linear.x))
            GPIO.output(self.left_dir_pin, 0)

        elif(msg.linear.x < 0.0):
            #right side
            self.pwm_right.ChangeDutyCycle(100*np.abs(msg.linear.x))
            GPIO.output(self.right_dir_pin, 0)

            #left side
            self.pwm_left.ChangeDutyCycle(100*np.abs(msg.linear.x))
            GPIO.output(self.left_dir_pin, 1)
            
        elif(msg.angular.z < 0.0):
            #right side
            self.pwm_right.ChangeDutyCycle(100*np.abs(msg.anglar.z))
            GPIO.output(self.right_dir_pin, 0)

            #left side
            self.pwm_left.ChangeDutyCycle(100*np.abs(msg.angular.z))
            GPIO.output(self.left_dir_pin, 0)

        elif(msg.angular.z > 0.0):
            #right side
            self.pwm_right.ChangeDutyCycle(100*np.abs(msg.angular.z))
            GPIO.output(self.right_dir_pin, 1)

            #left side
            self.pwm_left.ChangeDutyCycle(100*np.abs(msg.angular.z))
            GPIO.output(self.left_dir_pin, 1)
            
        else:
            self.pwm_left.ChangeDutyCycle(0)
            self.pwm_right.ChangeDutyCycle(0)
            
        # elif(np.abs(self.last_msg.twist.angular.z) > 0.1):
        #     GPIO.output(self.pins[x], self.last_msg)
        #     GPIO.outpit(self.dir_pins[x], x)




        

if __name__ == "__main__":
    motor_gpio()
