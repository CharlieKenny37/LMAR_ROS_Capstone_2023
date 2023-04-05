import RPi.GPIO as GPIO
import time

water_pin = 36
GPIO.setmode(GPIO.BOARD)
GPIO.setup(water_pin, GPIO.IN)

while(True):
    print(GPIO.input(water_pin))
    time.sleep(0.5)

