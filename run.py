import time
import RPi.GPIO as GPIO
import numpy as np
from functools import wraps

# Constants for motor pins
MOTOR_1_PIN_1 = 11
MOTOR_1_PIN_2 = 15
MOTOR_2_PIN_1 = 35
MOTOR_2_PIN_2 = 12

# Ultrasonic sensor pins
ULTRASONIC_FRONT_TRIGGER = 5
ULTRASONIC_FRONT_ECHO = 21

LIMIT_SWITCH_PIN = 3

TURN_ON_OFF_BUTTON_PIN = 7

FAN_PIN = 40

# Threshold distances in centimeters
TOO_CLOSE_FRONT = 10.0

# Time to turn 90 degrees (in seconds)
TURNING_TIME = 3.471
# PWM frequency
PWM_FREQ = 1000  # 1 kHz

# Room dimensions in cm (example values)
ROOM_WIDTH = 400
ROOM_HEIGHT = 300
ROBOT_DIAMETER = 40

# Grid size based on robot diameter
GRID_SIZE = ROBOT_DIAMETER

# Number of grid cells
NUM_CELLS_X = ROOM_WIDTH // GRID_SIZE
NUM_CELLS_Y = ROOM_HEIGHT // GRID_SIZE

# Create a grid to track visited cells
visited_grid = np.zeros((NUM_CELLS_X, NUM_CELLS_Y), dtype=bool)

# Disable GPIO warnings
GPIO.setwarnings(False)

def setup_gpio():
    GPIO.cleanup()
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([MOTOR_1_PIN_1, MOTOR_1_PIN_2, MOTOR_2_PIN_1, MOTOR_2_PIN_2, FAN_PIN], GPIO.OUT)
    GPIO.setup(ULTRASONIC_FRONT_TRIGGER, GPIO.OUT)
    GPIO.setup(ULTRASONIC_FRONT_ECHO, GPIO.IN)
    GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(TURN_ON_OFF_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def cleanup_gpio():
    GPIO.cleanup()
    #print("GPIO cleaned up")

def setup_pwm():
    global pwm_motor_2_pin_1, pwm_motor_2_pin_2
    pwm_motor_2_pin_1 = GPIO.PWM(MOTOR_2_PIN_1, PWM_FREQ)
    pwm_motor_2_pin_2 = GPIO.PWM(MOTOR_2_PIN_2, PWM_FREQ)
    pwm_motor_2_pin_1.start(0)
    pwm_motor_2_pin_2.start(0)
    #print("PWM setup complete")

def stop_motors_and_fan():
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, False)
    pwm_motor_2_pin_1.ChangeDutyCycle(0)
    pwm_motor_2_pin_2.ChangeDutyCycle(0)
    GPIO.output(FAN_PIN, False)
    #print("Motors and fan stopped")

def check_button(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        button_state = GPIO.input(TURN_ON_OFF_BUTTON_PIN)
        if button_state == 0:
            return func(*args, **kwargs)
        else:
            stop_motors_and_fan()
            #print("Button not pressed, action halted.")
            return None
    return wrapper

@check_button
def move_forward(speed=80):
    #print("Moving forward")
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    pwm_motor_2_pin_1.ChangeDutyCycle(speed)
    pwm_motor_2_pin_2.ChangeDutyCycle(0)

@check_button
def move_backward(speed=73):
    #print("Moving backward")
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    pwm_motor_2_pin_1.ChangeDutyCycle(0)
    pwm_motor_2_pin_2.ChangeDutyCycle(speed)

@check_button
def turn_left(speed=73):
    #print("Turning left")
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    pwm_motor_2_pin_1.ChangeDutyCycle(speed)
    pwm_motor_2_pin_2.ChangeDutyCycle(0)

@check_button
def turn_right(speed=73):
    #print("Turning right")
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    pwm_motor_2_pin_1.ChangeDutyCycle(0)
    pwm_motor_2_pin_2.ChangeDutyCycle(speed)

@check_button
def stop_motors():
    #print("Stopping motors")
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

def mark_cell_visited(x, y):
    if 0 <= x < NUM_CELLS_X and 0 <= y < NUM_CELLS_Y:
        visited_grid[x, y] = True

def all_cells_visited():
    return np.all(visited_grid)

@check_button
def turn_on_fan():
    GPIO.output(FAN_PIN, True)

def main():
    try:            
        last_turn = 'right'

        while True:
            turn_on_fan()
            limit_switch_state = GPIO.input(LIMIT_SWITCH_PIN)
            if limit_switch_state == 0:
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
        stop_motors_and_fan()
        cleanup_gpio()

def check_button_and_run():
    setup_gpio()
    setup_pwm()
    try:
        while True:
            button_state = GPIO.input(TURN_ON_OFF_BUTTON_PIN)
            #print(f"Button state: {button_state}")  # Debug statement
            if button_state == 0:
                #print("Button pressed, starting main loop.")
                main()
            else:
                stop_motors_and_fan()
                #print("Button not pressed, waiting.")
            time.sleep(1)  # Check button state every second
    except KeyboardInterrupt:
        cleanup_gpio()

if __name__ == "__main__":
    check_button_and_run()
