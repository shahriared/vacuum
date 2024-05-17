import time
import RPi.GPIO as GPIO

# Constants for motor pins
MOTOR_1_PIN_1 = 11
MOTOR_1_PIN_2 = 13
MOTOR_2_PIN_1 = 32
MOTOR_2_PIN_2 = 33

# Ultrasonic sensor pins
ULTRASONIC_LEFT_TRIGGER = 38
ULTRASONIC_LEFT_ECHO = 40
ULTRASONIC_FRONT_TRIGGER = 5
ULTRASONIC_FRONT_ECHO = 3
ULTRASONIC_RIGHT_TRIGGER = 23
ULTRASONIC_RIGHT_ECHO = 21

# Threshold distances in centimeters
TOO_CLOSE_WALL = 18.0
TOO_FAR_WALL = 25.0
TOO_CLOSE_FRONT = 10.0

# Time to turn 90 degrees (in seconds)
TURNING_TIME = 2.24

def setup_gpio():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([MOTOR_1_PIN_1, MOTOR_1_PIN_2, MOTOR_2_PIN_1, MOTOR_2_PIN_2], GPIO.OUT)
    GPIO.setup(ULTRASONIC_FRONT_TRIGGER, GPIO.OUT)
    GPIO.setup(ULTRASONIC_FRONT_ECHO, GPIO.IN)
    GPIO.setup(ULTRASONIC_LEFT_TRIGGER, GPIO.OUT)
    GPIO.setup(ULTRASONIC_LEFT_ECHO, GPIO.IN)
    GPIO.setup(ULTRASONIC_RIGHT_TRIGGER, GPIO.OUT)
    GPIO.setup(ULTRASONIC_RIGHT_ECHO, GPIO.IN)
    print("GPIO setup complete")

def cleanup_gpio():
    GPIO.cleanup()
    print("GPIO cleaned up")

def move_forward():
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    GPIO.output(MOTOR_2_PIN_1, True)
    GPIO.output(MOTOR_2_PIN_2, False)
    print("Moving forward")

def move_backward():
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    GPIO.output(MOTOR_2_PIN_1, False)
    GPIO.output(MOTOR_2_PIN_2, True)
    print("Moving backward")
    time.sleep(2)  # Move backward for 2 seconds

def turn_left():
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    GPIO.output(MOTOR_2_PIN_1, True)
    GPIO.output(MOTOR_2_PIN_2, False)
    print("Turning left")
    time.sleep(TURNING_TIME)  # Rotate left for 90 degrees

def turn_right():
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    GPIO.output(MOTOR_2_PIN_1, False)
    GPIO.output(MOTOR_2_PIN_2, True)
    print("Turning right")
    time.sleep(TURNING_TIME)  # Rotate right for 90 degrees

def stop_motors():
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, False)
    GPIO.output(MOTOR_2_PIN_1, False)
    GPIO.output(MOTOR_2_PIN_2, False)
    print("Motors stopped")

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

def main():
    try:
        setup_gpio()

        

        while True:
            move_forward()
            # print("Starting distance measurement")
            # front_distance = get_distance(ULTRASONIC_FRONT_TRIGGER, ULTRASONIC_FRONT_ECHO)

            # time.sleep(0.1)  # Sleep for 100 ms

            # print(f"Front: {front_distance} cm")

            # last_turn = 'right'

            # if front_distance < TOO_CLOSE_FRONT:
            #     stop_motors()
            #     if last_turn == 'right':
            #         turn_left()
            #         move_forward()
            #         time.sleep(2)
            #         turn_left()
            #         last_turn = 'left'
            #     else:
            #         turn_right()
            #         move_forward()
            #         time.sleep(2)
            #         turn_right()
            #         last_turn = 'right'
               
                
            # else:
            #     move_forward()  # Move forward normally

    except KeyboardInterrupt:
        pass
    finally:
        cleanup_gpio()

if __name__ == "__main__":
    main()
