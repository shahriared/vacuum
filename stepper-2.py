#!/usr/bin/python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

DIR = 18
STEP = 16
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)

GPIO.output(DIR, GPIO.HIGH)

number_of_steps = 200


for x in range(number_of_steps):
	GPIO.output(DIR, GPIO.HIGH)
	time.sleep(0.01)
	GPIO.output(DIR, GPIO.LOW)
	time.sleep(0.01)

GPIO.cleanup()