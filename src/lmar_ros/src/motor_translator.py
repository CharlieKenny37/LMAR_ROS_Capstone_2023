#!/usr/bin/env python

import rospy
import serial

ser = serial.Serial('/dev/ttyUSB0', baudrate=9600)

ser.open()

#we need to determine what we actually want for this"
msg = "0 "
ser.write(msg)


