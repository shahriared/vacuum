#!/usr/bin/python
import RPi.GPIO as GPIO, time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
GPIO.setwarnings(False)
GPIO.output(16, True)

#iemand anders 500
p = GPIO.PWM(16, 500)

def SpinMotor(direction, num_steps):
    p.ChangeFrequency(500)
    GPIO.output(18, direction)
    num_steps = float(num_steps)
    while num_steps > 0:
        p.start(1)
        time.sleep(0.01)
        num_steps -= 1
    p.stop()
    GPIO.cleanup()
    return True

direction_input = input('Left or Right (L/R):')
degree = input('Degree: ')
num_steps = 339.5/360
num_steps = float(degree) * num_steps
if direction_input == 'L':
    SpinMotor(True, num_steps)
else:
    SpinMotor(False, num_steps)