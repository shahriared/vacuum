import time
import RPi.GPIO as GPIO
import numpy as np

# Constants for motor pins
MOTOR_1_PIN_1 = 15
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

LIMIT_SWITCH_PIN_LEFT = 7
LIMIT_SWITCH_PIN_RIGHT = 8

# Threshold distances in centimeters
TOO_CLOSE_WALL = 18.0
TOO_FAR_WALL = 25.0
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

def setup_gpio():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([MOTOR_1_PIN_1, MOTOR_1_PIN_2, MOTOR_2_PIN_1, MOTOR_2_PIN_2], GPIO.OUT)
    GPIO.setup([ULTRASONIC_LEFT_TRIGGER, ULTRASONIC_RIGHT_TRIGGER, ULTRASONIC_FRONT_TRIGGER], GPIO.OUT)
    GPIO.setup([ULTRASONIC_LEFT_ECHO, ULTRASONIC_RIGHT_ECHO, ULTRASONIC_FRONT_ECHO], GPIO.IN)
    GPIO.setup([LIMIT_SWITCH_PIN_LEFT, LIMIT_SWITCH_PIN_RIGHT], GPIO.IN)
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
    print("Moving forward")
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    pwm_motor_2_pin_1.ChangeDutyCycle(speed)
    pwm_motor_2_pin_2.ChangeDutyCycle(0)

def move_backward(speed=73):
    print("Moving backward")
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    pwm_motor_2_pin_1.ChangeDutyCycle(0)
    pwm_motor_2_pin_2.ChangeDutyCycle(speed)

def turn_left(speed=73):
    print("Turning left")
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    pwm_motor_2_pin_1.ChangeDutyCycle(speed)
    pwm_motor_2_pin_2.ChangeDutyCycle(0)

def turn_right(speed=73):
    print("Turning right")
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    pwm_motor_2_pin_1.ChangeDutyCycle(0)
    pwm_motor_2_pin_2.ChangeDutyCycle(speed)

def stop_motors():
    print("Stopping motors")
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

def main():
    setup_gpio()
    setup_pwm()

    current_x = 0
    current_y = 0
    mark_cell_visited(current_x, current_y)

    print(not all_cells_visited())

    try:
        while not all_cells_visited():
            print(f"Current position: ({current_x}, {current_y})")
            left_limit_switch_state = GPIO.input(LIMIT_SWITCH_PIN_LEFT)
            right_limit_switch_state = GPIO.input(LIMIT_SWITCH_PIN_RIGHT)
            print(f"Left limit switch: {left_limit_switch_state}, Right limit switch: {right_limit_switch_state}")
            left_distance = get_distance(ULTRASONIC_LEFT_TRIGGER, ULTRASONIC_LEFT_ECHO)
            print(f"Left distance: {left_distance}")
            front_distance = get_distance(ULTRASONIC_FRONT_TRIGGER, ULTRASONIC_FRONT_ECHO)
            print(f"Front distance: {front_distance}")
            
            right_distance = get_distance(ULTRASONIC_RIGHT_TRIGGER, ULTRASONIC_RIGHT_ECHO)
            print(f"Right distance: {right_distance}")

            print(f"Front distance: {front_distance}, Left distance: {left_distance}, Right distance: {right_distance}")
            print(f"Left limit switch: {left_limit_switch_state}, Right limit switch: {right_limit_switch_state}")

            if left_limit_switch_state == 1 or right_limit_switch_state == 1 or front_distance < TOO_CLOSE_FRONT:
                print("Obstacle detected, moving backward and turning around")
                move_backward()
                time.sleep(1)
                turn_left()
                time.sleep(TURNING_TIME)
                turn_left()
                time.sleep(TURNING_TIME)
                current_x = max(current_x - 1, 0)  # Move to previous grid cell
            elif left_distance < TOO_CLOSE_WALL:
                print("Too close to left wall, turning right")
                turn_right()
                time.sleep(TURNING_TIME / 2)
            elif right_distance < TOO_CLOSE_WALL:
                print("Too close to right wall, turning left")
                turn_left()
                time.sleep(TURNING_TIME / 2)
            else:
                print("Moving forward")
                move_forward()
                time.sleep(1)
                current_y += 1  # Move to next grid cell
                mark_cell_visited(current_x, current_y)
    except KeyboardInterrupt:
        pass
    finally:
        stop_motors()
        cleanup_gpio()

if __name__ == "__main__":
    main()
