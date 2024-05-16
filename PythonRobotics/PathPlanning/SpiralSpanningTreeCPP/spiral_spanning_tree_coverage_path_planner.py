import time
import RPi.GPIO as GPIO

# Constants for motor control pins
MOTOR_1_PIN_1 = 11
MOTOR_1_PIN_2 = 13
MOTOR_2_PIN_1 = 16
MOTOR_2_PIN_2 = 18

# Ultrasonic sensor pins
ULTRASONIC_LEFT_TRIGGER = 3
ULTRASONIC_LEFT_ECHO = 5
ULTRASONIC_FRONT_TRIGGER = 6
ULTRASONIC_FRONT_ECHO = 9
ULTRASONIC_RIGHT_TRIGGER = 10
ULTRASONIC_RIGHT_ECHO = 11

# Threshold distances (in cm)
DISTANCE_THRESHOLD = 20

def setup_gpio():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(MOTOR_1_PIN_1, GPIO.OUT)
    GPIO.setup(MOTOR_1_PIN_2, GPIO.OUT)
    GPIO.setup(MOTOR_2_PIN_1, GPIO.OUT)
    GPIO.setup(MOTOR_2_PIN_2, GPIO.OUT)
    GPIO.setup(ULTRASONIC_LEFT_TRIGGER, GPIO.OUT)
    GPIO.setup(ULTRASONIC_LEFT_ECHO, GPIO.IN)
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

def stop():
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, False)
    GPIO.output(MOTOR_2_PIN_1, False)
    GPIO.output(MOTOR_2_PIN_2, False)

def get_distance(trigger_pin, echo_pin):
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    pulse_start = time.time()
    pulse_end = time.time()

    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()

    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound in cm/s
    distance = round(distance, 2)  # Round to 2 decimal places
    return distance

def main():
    setup_gpio()

    try:
        while True:
            distance_left = get_distance(ULTRASONIC_LEFT_TRIGGER, ULTRASONIC_LEFT_ECHO)
            distance_front = get_distance(ULTRASONIC_FRONT_TRIGGER, ULTRASONIC_FRONT_ECHO)
            distance_right = get_distance(ULTRASONIC_RIGHT_TRIGGER, ULTRASONIC_RIGHT_ECHO)

            print("Distance Left:", distance_left, "cm")
            print("Distance Front:", distance_front, "cm")
            print("Distance Right:", distance_right, "cm")

            if ((distance_left <= DISTANCE_THRESHOLD and distance_front > DISTANCE_THRESHOLD and distance_right <= DISTANCE_THRESHOLD) or
                (distance_left > DISTANCE_THRESHOLD and distance_front > DISTANCE_THRESHOLD and distance_right > DISTANCE_THRESHOLD)):
                move_forward()

            elif ((distance_left <= DISTANCE_THRESHOLD and distance_front <= DISTANCE_THRESHOLD and distance_right > DISTANCE_THRESHOLD) or
                  (distance_left <= DISTANCE_THRESHOLD and distance_front > DISTANCE_THRESHOLD and distance_right > DISTANCE_THRESHOLD)):
                stop()
                turn_left()
                time.sleep(0.1)

            elif ((distance_left > DISTANCE_THRESHOLD and distance_front <= DISTANCE_THRESHOLD and distance_right <= DISTANCE_THRESHOLD) or
                  (distance_left > DISTANCE_THRESHOLD and distance_front > DISTANCE_THRESHOLD and distance_right <= DISTANCE_THRESHOLD) or
                  (distance_left > DISTANCE_THRESHOLD and distance_front <= DISTANCE_THRESHOLD and distance_right > DISTANCE_THRESHOLD)):
                stop()
                turn_right()
                time.sleep(0.1)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Exiting program...")

    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
