import os
import sys
import math

import time

import numpy as np
import matplotlib.pyplot as plt

import time
import RPi.GPIO as GPIO

from gpiozero import DistanceSensor


# Constants
MOTOR_1_PIN_1 = 11  # Left wheel
MOTOR_1_PIN_2 = 13  # Left wheel
MOTOR_2_PIN_1 = 16  # Right wheel
MOTOR_2_PIN_2 = 18  # Right wheel

# Ultrasonic sensors pins
ULTRASONIC_FRONT_TRIGGER = 16
ULTRASONIC_FRONT_ECHO = 18
ULTRASONIC_RIGHT_TRIGGER = 23
ULTRASONIC_RIGHT_ECHO = 21

# Ultrasonic sensors pins
# ULTRASONIC_TRIGGER = [16, 19, 21, 23]
# ULTRASONIC_ECHO = [18, 22, 24, 26]

# Threshold distance
DISTANCE_THRESHOLD = 10  # in cm

# Turning time for 90-degree rotation (in seconds)
TURNING_TIME = 2.24

# do_animation = False

def setup_gpio():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(MOTOR_1_PIN_1, GPIO.OUT)
    GPIO.setup(MOTOR_1_PIN_2, GPIO.OUT)
    GPIO.setup(MOTOR_2_PIN_1, GPIO.OUT)
    GPIO.setup(MOTOR_2_PIN_2, GPIO.OUT)
    GPIO.setup(ULTRASONIC_FRONT_TRIGGER, GPIO.OUT)
    GPIO.setup(ULTRASONIC_FRONT_ECHO, GPIO.IN)
    GPIO.setup(ULTRASONIC_RIGHT_TRIGGER, GPIO.OUT)
    GPIO.setup(ULTRASONIC_RIGHT_ECHO, GPIO.IN)

def move_forward():
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    GPIO.output(MOTOR_2_PIN_1, True)
    GPIO.output(MOTOR_2_PIN_2, False)

def move_backward():
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    GPIO.output(MOTOR_2_PIN_1, False)
    GPIO.output(MOTOR_2_PIN_2, True)

def turn_left():
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    GPIO.output(MOTOR_2_PIN_1, True)
    GPIO.output(MOTOR_2_PIN_2, False)

def turn_right():
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    GPIO.output(MOTOR_2_PIN_1, False)
    GPIO.output(MOTOR_2_PIN_2, True)

def get_distance(trigger_pin, echo_pin):
    # Send ultrasonic signal
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    # Wait for echo
    pulse_start = time.time()
    pulse_end = time.time()
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()

    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()

    # Calculate distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound is 343m/s, but we're measuring round trip
    distance = round(distance, 2)  # Round to 2 decimal places
    return distance

def main():
    setup_gpio()

    while True:
        move_forward()
        # Check distance from the front ultrasonic sensor
        distance_front = get_distance(ULTRASONIC_FRONT_TRIGGER, ULTRASONIC_FRONT_ECHO)
        print("Distance from Front Sensor:", distance_front, "cm")

        if distance_front > DISTANCE_THRESHOLD:
            # Move forward
            move_forward()
        else:
            # Stop and turn right for 90 degrees
            move_backward()
            time.sleep(TURNING_TIME)
            turn_right()  # Turn right

            # Follow the wall using the right ultrasonic sensor
            while True:
                distance_right = get_distance(ULTRASONIC_RIGHT_TRIGGER, ULTRASONIC_RIGHT_ECHO)
                print("Distance from Right Sensor:", distance_right, "cm")

                if distance_right <= DISTANCE_THRESHOLD:
                    # Move forward and maintain distance to the wall
                    move_forward()
                else:
                    # Turn right to adjust distance to the wall
                    turn_right()

                time.sleep(0.1)

    # try:
        # while True:
        #     move_forward()
        #     # Check distance from the front ultrasonic sensor
        #     distance_front = get_distance(ULTRASONIC_FRONT_TRIGGER, ULTRASONIC_FRONT_ECHO)
        #     print("Distance from Front Sensor:", distance_front, "cm")

        #     if distance_front > DISTANCE_THRESHOLD:
        #         # Move forward
        #         move_forward()
        #     else:
        #         # Stop and turn right for 90 degrees
        #         move_backward()
        #         time.sleep(TURNING_TIME)
        #         turn_right()  # Turn right

        #         # Follow the wall using the right ultrasonic sensor
        #         while True:
        #             distance_right = get_distance(ULTRASONIC_RIGHT_TRIGGER, ULTRASONIC_RIGHT_ECHO)
        #             print("Distance from Right Sensor:", distance_right, "cm")

        #             if distance_right <= DISTANCE_THRESHOLD:
        #                 # Move forward and maintain distance to the wall
        #                 move_forward()
        #             else:
        #                 # Turn right to adjust distance to the wall
        #                 turn_right()

        #             time.sleep(0.1)  # Adjust delay as needed

    # except KeyboardInterrupt:
    #     print("Exiting program...")
    # finally:
    #     GPIO.cleanup()

if __name__ == "__main__":
    main()
