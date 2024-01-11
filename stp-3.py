#!/usr/bin/python
import RPi.GPIO as GPIO
import time

# Set the GPIO pin numbers (adjust these based on your wiring)
step_pin = 16  # Connect this to the step pin on your stepper motor driver
dir_pin = 18   # Connect this to the direction pin on your stepper motor driver

# Set the mode to Broadcom SOC channel numbering
GPIO.setmode(GPIO.BCM)

# Set up the GPIO pins
GPIO.setup(step_pin, GPIO.OUT)
GPIO.setup(dir_pin, GPIO.OUT)

# Function to move the stepper motor
def move_stepper(steps, direction, delay):
    # Set the direction
    GPIO.output(dir_pin, direction)

    # Step the motor
    for _ in range(steps):
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(step_pin, GPIO.LOW)
        time.sleep(delay)

# Move the stepper motor 200 steps in one direction with a delay of 0.001 seconds between steps
move_stepper(200, GPIO.HIGH, 0.001)

# Pause for a moment
time.sleep(1)

# Move the stepper motor 200 steps in the opposite direction with a delay of 0.001 seconds between steps
move_stepper(200, GPIO.LOW, 0.001)

# Clean up GPIO resources
GPIO.cleanup()
