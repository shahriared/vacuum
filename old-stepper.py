#!/usr/bin/python
import RPi.GPIO as GPIO
import time

# Constants
MOTOR_PIN = 16
DIRECTION_PIN = 18
PWM_FREQUENCY = 500

def setup_gpio():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(MOTOR_PIN, GPIO.OUT)
    GPIO.setup(DIRECTION_PIN, GPIO.OUT)
    GPIO.output(MOTOR_PIN, True)
    return GPIO.PWM(MOTOR_PIN, PWM_FREQUENCY)

def spin_motor(direction, num_steps):
    p = setup_gpio()
    p.ChangeFrequency(PWM_FREQUENCY)
    GPIO.output(DIRECTION_PIN, direction)
    num_steps = float(num_steps)
    while num_steps > 0:
        p.start(1)
        time.sleep(0.01)
        num_steps -= 1
    p.stop()

def get_user_input(prompt):
    while True:
        user_input = input(prompt).strip().upper()
        if user_input in ['L', 'R']:
            return user_input
        print("Invalid input. Please enter 'L' or 'R'.")

if __name__ == "__main__":
    direction_input = get_user_input('Left or Right (L/R): ')
    degree = float(input('Degree: '))
    num_steps = (339.5 / 360) * degree
    if direction_input == 'L':
        spin_motor(True, num_steps)
    else:
        spin_motor(False, num_steps)
    GPIO.cleanup()
