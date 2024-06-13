import time
import RPi.GPIO as GPIO
import argparse
import keyboard

# Constants for motor pins
MOTOR_1_PIN_1 = 13
MOTOR_1_PIN_2 = 11
MOTOR_2_PIN_1 = 12
MOTOR_2_PIN_2 = 35

# Ultrasonic sensor pins
ULTRASONIC_LEFT_TRIGGER = 38
ULTRASONIC_LEFT_ECHO = 40
ULTRASONIC_FRONT_TRIGGER = 5
ULTRASONIC_FRONT_ECHO = 3
ULTRASONIC_RIGHT_TRIGGER = 23
ULTRASONIC_RIGHT_ECHO = 21

# Limit switch pins
LIMIT_SWITCH_PIN_1 = 7
LIMIT_SWITCH_PIN_2 = 15

# FAN_PIN = 8

# Threshold distances in centimeters
TOO_CLOSE_WALL = 18.0
TOO_FAR_WALL = 25.0
TOO_CLOSE_FRONT = 10.0

# Time to turn 90 degrees (in seconds)
TURNING_TIME = 3

# PWM frequency
PWM_FREQ = 1000  # 1 kHz

def setup_gpio():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([MOTOR_1_PIN_1, MOTOR_1_PIN_2, MOTOR_2_PIN_1, MOTOR_2_PIN_2], GPIO.OUT)
    GPIO.setup(ULTRASONIC_FRONT_TRIGGER, GPIO.OUT)
    GPIO.setup(ULTRASONIC_FRONT_ECHO, GPIO.IN)
    GPIO.setup(ULTRASONIC_LEFT_TRIGGER, GPIO.OUT)
    GPIO.setup(ULTRASONIC_LEFT_ECHO, GPIO.IN)
    GPIO.setup(ULTRASONIC_RIGHT_TRIGGER, GPIO.OUT)
    GPIO.setup(ULTRASONIC_RIGHT_ECHO, GPIO.IN)
    GPIO.setup(LIMIT_SWITCH_PIN_1, GPIO.IN)
    GPIO.setup(LIMIT_SWITCH_PIN_2, GPIO.IN)
    # GPIO.setup(FAN_PIN, GPIO.OUT)
    print("GPIO setup complete")

def cleanup_gpio():
    GPIO.cleanup()
    print("GPIO cleaned up")

def setup_pwm():
    global pwm_motor_2_pin_1, pwm_motor_2_pin_2
    pwm_motor_2_pin_1 = GPIO.PWM(MOTOR_2_PIN_1, PWM_FREQ)
    pwm_motor_2_pin_2 = GPIO.PWM(MOTOR_2_PIN_2, PWM_FREQ)
    pwm_motor_2_pin_1.start(0)
    pwm_motor_2_pin_2.start(0)
    print("PWM setup complete")

def move_forward(speed=73):
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    pwm_motor_2_pin_1.ChangeDutyCycle(speed)
    pwm_motor_2_pin_2.ChangeDutyCycle(0)

def move_backward(speed=73):
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    pwm_motor_2_pin_1.ChangeDutyCycle(0)
    pwm_motor_2_pin_2.ChangeDutyCycle(speed)

def turn_left(speed=73):
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    pwm_motor_2_pin_1.ChangeDutyCycle(speed)
    pwm_motor_2_pin_2.ChangeDutyCycle(0)

def turn_right(speed=73):
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    pwm_motor_2_pin_1.ChangeDutyCycle(0)
    pwm_motor_2_pin_2.ChangeDutyCycle(speed)

def stop_motors():
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, False)
    pwm_motor_2_pin_1.ChangeDutyCycle(0)
    pwm_motor_2_pin_2.ChangeDutyCycle(0)

def get_distance(trigger_pin, echo_pin):
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    pulse_start = time.time()
    pulse_end = time.time()

    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
        if time.time() - pulse_start > 0.04:  # Timeout after 40 ms
            return -1

    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()
        if time.time() - pulse_end > 0.04:  # Timeout after 40 ms
            return -1

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound in cm/s
    distance = round(distance, 2)
    return distance

# def turn_fan_on():
#     GPIO.output(FAN_PIN, True)
#     print("Fan turned on")


def main():
    setup_gpio()
    setup_pwm()

    try:            
        last_turn = 'right'

        while True:
            limit_switch_state_1 = GPIO.input(LIMIT_SWITCH_PIN_1)
            limit_switch_state_2 = GPIO.input(LIMIT_SWITCH_PIN_2)
            if limit_switch_state_1 == 1 or limit_switch_state_2 == 1:
                move_backward()
                time.sleep(1)
                turn_left()
                time.sleep(TURNING_TIME)
                turn_left()
                time.sleep(TURNING_TIME)
            else:
                front_distance = get_distance(ULTRASONIC_FRONT_TRIGGER, ULTRASONIC_FRONT_ECHO)
                left_distance = get_distance(ULTRASONIC_LEFT_TRIGGER, ULTRASONIC_LEFT_ECHO)
                right_distance = get_distance(ULTRASONIC_RIGHT_TRIGGER, ULTRASONIC_RIGHT_ECHO)

                time.sleep(0.1)

                if front_distance < TOO_CLOSE_FRONT:
                    stop_motors()
                    if last_turn == 'right':
                        turn_left()
                        time.sleep(TURNING_TIME)
                        move_forward()
                        time.sleep(2)
                        turn_left()
                        time.sleep(TURNING_TIME)
                        last_turn = 'left'
                    else:
                        turn_right()
                        time.sleep(TURNING_TIME)
                        move_forward()
                        time.sleep(2)
                        turn_right()
                        time.sleep(TURNING_TIME)
                        last_turn = 'right'
                elif left_distance < TOO_CLOSE_WALL:
                    stop_motors()
                    turn_right()
                    time.sleep(TURNING_TIME)
                    last_turn = 'right'
                elif right_distance < TOO_CLOSE_WALL:
                    stop_motors()
                    turn_left()
                    time.sleep(TURNING_TIME)
                    last_turn = 'left'
                else:
                    move_forward()  # Move forward normally
    except KeyboardInterrupt:
        pass
    finally:
        stop_motors()
        cleanup_gpio()

if __name__ == "__main__":
    main()
