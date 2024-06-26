import time
import RPi.GPIO as GPIO

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

# Create PWM instances for MOTOR_2 pins
def setup_pwm():
    global pwm_motor_2_pin_1, pwm_motor_2_pin_2
    pwm_motor_2_pin_1 = GPIO.PWM(MOTOR_2_PIN_1, PWM_FREQ)
    pwm_motor_2_pin_2 = GPIO.PWM(MOTOR_2_PIN_2, PWM_FREQ)
    pwm_motor_2_pin_1.start(0)  # Start with 0% duty cycle
    pwm_motor_2_pin_2.start(0)  # Start with 0% duty cycle
    print("PWM setup complete")

def move_forward(speed=73):
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    pwm_motor_2_pin_1.ChangeDutyCycle(speed)
    pwm_motor_2_pin_2.ChangeDutyCycle(0)
    print(f"Moving forward with speed {speed}%")

def move_backward(speed=73):
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    pwm_motor_2_pin_1.ChangeDutyCycle(0)
    pwm_motor_2_pin_2.ChangeDutyCycle(speed)
    # print(f"Moving backward with speed {speed}%")

def turn_left(speed=73):
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    pwm_motor_2_pin_1.ChangeDutyCycle(speed)
    pwm_motor_2_pin_2.ChangeDutyCycle(0)
    print(f"Turning left with speed {speed}%")
    time.sleep(TURNING_TIME)  # Rotate left for 90 degrees

def turn_right(speed=73):
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    pwm_motor_2_pin_1.ChangeDutyCycle(0)
    pwm_motor_2_pin_2.ChangeDutyCycle(speed)
    print(f"Turning right with speed {speed}%")
    time.sleep(TURNING_TIME)  # Rotate right for 90 degrees

def stop_motors():
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, False)
    pwm_motor_2_pin_1.ChangeDutyCycle(0)
    pwm_motor_2_pin_2.ChangeDutyCycle(0)
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

def turn_fan_on():
    GPIO.output(FAN_PIN, True)
    print("Fan turned on")

def main():
    try:
        setup_gpio()
        setup_pwm()

        turn_fan_on()
            
        last_turn = 'right'

        while True:
            limit_switch_state = GPIO.input(LIMIT_SWITCH_PIN)
            if limit_switch_state == 1:
                move_backward()
                time.sleep(1)
                turn_left()
                turn_left()
            else:
                front_distance = get_distance(ULTRASONIC_FRONT_TRIGGER, ULTRASONIC_FRONT_ECHO)

                time.sleep(0.1)  # Sleep for 100 ms

                print(f"Front: {front_distance} cm")

                if front_distance < TOO_CLOSE_FRONT:
                    stop_motors()
                    if last_turn == 'right':
                        turn_left()
                        move_forward()
                        time.sleep(2)
                        turn_left()
                        last_turn = 'left'
                    else:
                        turn_right()
                        move_forward()
                        time.sleep(2)
                        turn_right()
                        last_turn = 'right'
                else:
                    move_forward()  # Move forward normally
    except KeyboardInterrupt:
        pass
    finally:
        stop_motors()
        cleanup_gpio()

if __name__ == "__main__":
    main()
