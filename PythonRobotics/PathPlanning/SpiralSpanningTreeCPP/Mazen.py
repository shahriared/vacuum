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

# Threshold distances in centimeters
TOO_CLOSE_WALL = 18.0
TOO_FAR_WALL = 25.0
TOO_CLOSE_FRONT = 10.0

# Time to turn 90 degrees (in seconds)
TURNING_TIME = 2

def setup_gpio():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([MOTOR_1_PIN_1, MOTOR_1_PIN_2, MOTOR_2_PIN_1, MOTOR_2_PIN_2], GPIO.OUT)
    GPIO.setup(ULTRASONIC_FRONT_TRIGGER, GPIO.OUT)
    GPIO.setup(ULTRASONIC_FRONT_ECHO, GPIO.IN)
    GPIO.setup(ULTRASONIC_LEFT_TRIGGER, GPIO.OUT)
    GPIO.setup(ULTRASONIC_LEFT_ECHO, GPIO.IN)
    GPIO.setup(ULTRASONIC_RIGHT_TRIGGER, GPIO.OUT)
    GPIO.setup(ULTRASONIC_RIGHT_ECHO, GPIO.IN)
    motor_pwm_1 = GPIO.PWM(MOTOR_2_PIN_1, 100)  # 100 Hz frequency
    motor_pwm_2 = GPIO.PWM(MOTOR_2_PIN_2, 100)
    motor_pwm_1.start(0)  # Start with 0% duty cycle (stopped)
    motor_pwm_2.start(0)
    print("GPIO setup complete")

def cleanup_gpio():
    GPIO.cleanup()
    print("GPIO cleaned up")

def move_forward():
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    GPIO.output(MOTOR_2_PIN_1, True)
    GPIO.output(MOTOR_2_PIN_2, False)
    motor_pwm_1.ChangeDutyCycle(speed_percen=75)
    motor_pwm_2.ChangeDutyCycle(speed_percent=50)
    GPIO.output(MOTOR_1_PIN_2, False)
    GPIO.output(MOTOR_2_PIN_2, False)
    print(f"Moving forward at {speed_percent}% speed")

def move_backward():
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    GPIO.output(MOTOR_2_PIN_1, False)
    GPIO.output(MOTOR_2_PIN_2, True)
    print("Moving backward")

def turn_left():
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    GPIO.output(MOTOR_2_PIN_1, True)
    GPIO.output(MOTOR_2_PIN_2, False)
    motor_pwm_1.ChangeDutyCycle(speed_percent=75)
    motor_pwm_2.ChangeDutyCycle(speed_percent=60)
    GPIO.output(MOTOR_1_PIN_2, True)
    GPIO.output(MOTOR_2_PIN_2, False)
    print(f"Turning left at {speed_percent}% speed")

def turn_right():
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    GPIO.output(MOTOR_2_PIN_1, False)
    GPIO.output(MOTOR_2_PIN_2, True)
    motor_pwm_1.ChangeDutyCycle(speed_percent=75)
    motor_pwm_2.ChangeDutyCycle(speed_percent=50)
    GPIO.output(MOTOR_1_PIN_2, False)
    GPIO.output(MOTOR_2_PIN_2, True)
    print(f"Turning right at {speed_percent}% speed")

def stop_motors():
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, False)
    GPIO.output(MOTOR_2_PIN_1, False)
    GPIO.output(MOTOR_2_PIN_2, False)
    motor_pwm_1.ChangeDutyCycle(0)  # Stop by setting duty cycle to 0%
    motor_pwm_2.ChangeDutyCycle(0)
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
            print("Starting distance measurement")
            front_distance = get_distance(ULTRASONIC_FRONT_TRIGGER, ULTRASONIC_FRONT_ECHO)
            left_distance = get_distance(ULTRASONIC_LEFT_TRIGGER, ULTRASONIC_LEFT_ECHO)
            right_distance = get_distance(ULTRASONIC_RIGHT_TRIGGER, ULTRASONIC_RIGHT_ECHO)

            print(f"Front: {front_distance} cm")
            print(f"Left: {left_distance} cm")
            print(f"Right: {right_distance} cm")

            

    except KeyboardInterrupt:
        pass
    finally:
        cleanup_gpio()

if __name__ == "__main__":
    main()

