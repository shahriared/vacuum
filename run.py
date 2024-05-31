import time
import RPi.GPIO as GPIO
import argparse
from pynput import keyboard

# Constants for motor pins
MOTOR_1_PIN_1 = 11
MOTOR_1_PIN_2 = 13
MOTOR_2_PIN_1 = 12
MOTOR_2_PIN_2 = 35

# Ultrasonic sensor pins
ULTRASONIC_LEFT_TRIGGER = 38
ULTRASONIC_LEFT_ECHO = 40
ULTRASONIC_FRONT_TRIGGER = 5
ULTRASONIC_FRONT_ECHO = 3
ULTRASONIC_RIGHT_TRIGGER = 23
ULTRASONIC_RIGHT_ECHO = 21

LIMIT_SWITCH_PIN = 7

FAN_PIN = 8

# Threshold distances in centimeters
TOO_CLOSE_WALL = 18.0
TOO_FAR_WALL = 25.0
TOO_CLOSE_FRONT = 10.0

# Time to turn 90 degrees (in seconds)
TURNING_TIME = 2.6

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
    GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN)
    GPIO.setup(FAN_PIN, GPIO.OUT)
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

def turn_fan_on():
    GPIO.output(FAN_PIN, True)
    print("Fan turned on")

def automatic_mode():
    try:            
        last_turn = 'right'

        while True:
            limit_switch_state = GPIO.input(LIMIT_SWITCH_PIN)
            if limit_switch_state == 1:
                move_backward()
                time.sleep(1)
                turn_left()
                time.sleep(TURNING_TIME)
                turn_left()
                time.sleep(TURNING_TIME)
            else:
                front_distance = get_distance(ULTRASONIC_FRONT_TRIGGER, ULTRASONIC_FRONT_ECHO)

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
                else:
                    move_forward()  # Move forward normally
    except KeyboardInterrupt:
        pass
    finally:
        stop_motors()
        cleanup_gpio()

class KeyboardController:
    def __init__(self):
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()
        self.current_action = None

    def on_press(self, key):
        try:
            if key.char == ' ':
                self.stop()
        except AttributeError:
            if key == keyboard.Key.up:
                self.current_action = move_forward
                self.current_action()
            elif key == keyboard.Key.down:
                self.current_action = move_backward
                self.current_action()
            elif key == keyboard.Key.left:
                self.current_action = turn_left
                self.current_action()
            elif key == keyboard.Key.right:
                self.current_action = turn_right
                self.current_action()

    def on_release(self, key):
        if key in (keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right):
            self.stop()

    def stop(self):
        if self.current_action:
            stop_motors()
            self.current_action = None

def keyboard_control():
    print("Keyboard control mode. Use arrow keys to control the robot and space to stop.")
    controller = KeyboardController()
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

def main():
    parser = argparse.ArgumentParser(description='Robot control script.')
    parser.add_argument('--control-with-keyboard', action='store_true', help='Control the robot with the keyboard')
    args = parser.parse_args()

    try:
        setup_gpio()
        setup_pwm()

        turn_fan_on()

        if args.control_with_keyboard:
            keyboard_control()
        else:
            automatic_mode()
    finally:
        stop_motors()
        cleanup_gpio()

if __name__ == "__main__":
    main()
