#!/usr/bin/python
import RPi.GPIO as GPIO, time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
GPIO.setwarnings(False)
GPIO.output(16, True)




def SpinMotor(direction, num_steps):
	p = GPIO.PWM(16, 500)
	num_steps = int(num_steps);
#	p.ChangeFrequency(500)
#	print("number of steps")
#	print(num_steps)
	GPIO.output(18, direction)
	while num_steps > 0:
		#print("number of steps")
		#print(num_steps)
		p.start(1)
		time.sleep(0.001)
		num_steps -= 1
	p.stop()
	GPIO.cleanup()
	return True

direction_input = input('Please enter O or C fro Open or Close:')
num_steps = input('Please enter the number of steps: ')
if direction_input == 'C':
    SpinMotor(False, num_steps)
else:
    SpinMotor(True, num_steps)
