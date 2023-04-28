#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import numpy as np
from geometry_msgs.msg import Twist

class motor_gpio():
    
    
    def __init__(self):
        rospy.init_node("motor_controller")
        #self.last_time = rospy.time()
        #self.pins = [32, 33]
        #self.dir_pins = []
        GPIO.setmode(GPIO.BOARD)
        #for x in self.pins:
        #    GPIO.setup(x, GPIO.OUT)

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
        
        #pwm = GPIO.PWM(32, 20)
        #pwm.start(0)
        #pwm.ChangeDutyCycle(100)
        while(not rospy.is_shutdown()):
            ""
            #pwm.ChangeDutyCycle(100)
            #if(rospy.time() - self.last_time < 1):
            #    for x in np.size(self.pins):
            #        GPIO.output(self.pins[x], self.last_msg)
            #        GPIO.outpit(self.dir_pins[x], 1)
            #else:
            #    for x in self.pins:
            #        GPIO.output(self.pins[x], 0)
            #        GPIO.outpit(self.dir_pins[x], 0)
        
        #pwm.ChangeDutyCycle(0)
        #GPIO.output(36,0)
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
        # self.last_time = rospy.time()
        # self.last_msg = 255
        rospy.logerr("Linear X:")
        rospy.logerr(msg.linear.x)
        rospy.logerr("Angular Z:")
        rospy.logerr(msg.angular.z)
        if(msg.linear.x > 0.3):
            #right side
            self.pwm_right.ChangeDutyCycle(100)
            GPIO.output(self.right_dir_pin, 1)

            #left side
            self.pwm_left.ChangeDutyCycle(100)
            GPIO.output(self.left_dir_pin, 0)

        elif(msg.linear.x < -0.3):
            #right side
            self.pwm_right.ChangeDutyCycle(100)
            GPIO.output(self.right_dir_pin, 0)

            #left side
            self.pwm_left.ChangeDutyCycle(100)
            GPIO.output(self.left_dir_pin, 1)
            
        elif(msg.angular.z < -0.3):
            #right side
            self.pwm_right.ChangeDutyCycle(100)
            GPIO.output(self.right_dir_pin, 0)

            #left side
            self.pwm_left.ChangeDutyCycle(100)
            GPIO.output(self.left_dir_pin, 0)

        elif(msg.angular.z > 0.3):
            #right side
            self.pwm_right.ChangeDutyCycle(100)
            GPIO.output(self.right_dir_pin, 1)

            #left side
            self.pwm_left.ChangeDutyCycle(100)
            GPIO.output(self.left_dir_pin, 1)
            
        else:
            self.pwm_left.ChangeDutyCycle(0)
            self.pwm_right.ChangeDutyCycle(0)
            
        # elif(np.abs(self.last_msg.twist.angular.z) > 0.1):
        #     GPIO.output(self.pins[x], self.last_msg)
        #     GPIO.outpit(self.dir_pins[x], x)




        

if __name__ == "__main__":
    motor_gpio()
