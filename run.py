import time
import RPi.GPIO as GPIO
import numpy as np

# Constants for motor pins
MOTOR_1_PIN_1 = 15
MOTOR_1_PIN_2 = 11
MOTOR_2_PIN_1 = 12
MOTOR_2_PIN_2 = 35

# Ultrasonic sensor pins
ULTRASONIC_FRONT_TRIGGER = 5
ULTRASONIC_FRONT_ECHO = 21

STOP_BUTTON_PIN = 38
FAN_PIN = 40

# Threshold distances in centimeters
TOO_CLOSE_FRONT = 10.0

# Time to turn 90 degrees (in seconds)
TURNING_TIME = 3

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

# Toggle variable
is_stopped = True

# Disable GPIO warnings
GPIO.setwarnings(False)

def toggle_stop(channel):
    global is_stopped
    is_stopped = not is_stopped
    if not is_stopped:
        stop_motors()

def setup_gpio():
    GPIO.cleanup()
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([MOTOR_1_PIN_1, MOTOR_1_PIN_2, MOTOR_2_PIN_1, MOTOR_2_PIN_2, FAN_PIN], GPIO.OUT)
    GPIO.setup(ULTRASONIC_FRONT_TRIGGER, GPIO.OUT)
    GPIO.setup(ULTRASONIC_FRONT_ECHO, GPIO.IN)
    GPIO.setup(STOP_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    try:
        GPIO.add_event_detect(STOP_BUTTON_PIN, GPIO.FALLING, callback=toggle_stop, bouncetime=200)
    except RuntimeError as e:
        return False

def cleanup_gpio():
    GPIO.cleanup()

def setup_pwm():
    global pwm_motor_2_pin_1, pwm_motor_2_pin_2
    pwm_motor_2_pin_1 = GPIO.PWM(MOTOR_2_PIN_1, PWM_FREQ)
    pwm_motor_2_pin_2 = GPIO.PWM(MOTOR_2_PIN_2, PWM_FREQ)
    pwm_motor_2_pin_1.start(0)
    pwm_motor_2_pin_2.start(0)

def move_forward(speed=73):
    if is_stopped:
        stop_motors()
        return
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    pwm_motor_2_pin_1.ChangeDutyCycle(speed)
    pwm_motor_2_pin_2.ChangeDutyCycle(0)

def move_backward(speed=73):
    if is_stopped:
        stop_motors()
        return
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    pwm_motor_2_pin_1.ChangeDutyCycle(0)
    pwm_motor_2_pin_2.ChangeDutyCycle(speed)

def turn_left(speed=73):
    if is_stopped:
        stop_motors()
        return
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    pwm_motor_2_pin_1.ChangeDutyCycle(speed)
    pwm_motor_2_pin_2.ChangeDutyCycle(0)

def turn_right(speed=73):
    if is_stopped:
        stop_motors()
        return
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

def mark_cell_visited(x, y):
    if 0 <= x < NUM_CELLS_X and 0 <= y < NUM_CELLS_Y:
        visited_grid[x, y] = True

def all_cells_visited():
    return np.all(visited_grid)

def turn_on_fan():
    if is_stopped:
        stop_motors()
        return
    GPIO.output(FAN_PIN, True)

def main():
    setup_gpio()
    setup_pwm()

    try:
        last_turn = 'right'

        while True:
            if is_stopped:
                stop_motors()
                time.sleep(0.1)
                continue

            turn_on_fan()
            
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
                move_forward()

    except KeyboardInterrupt:
        pass
    finally:
        stop_motors()
        cleanup_gpio()

if __name__ == "__main__":
    main()
