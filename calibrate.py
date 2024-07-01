import time
import RPi.GPIO as GPIO

# Constants for motor pins
MOTOR_1_PIN_1 = 11
MOTOR_1_PIN_2 = 15
MOTOR_2_PIN_1 = 35
MOTOR_2_PIN_2 = 12

# PWM frequency
PWM_FREQ = 1000  # 1 kHz

# Default values
DEFAULT_SPEED = 73
DEFAULT_TURNING_TIME = 3

# Disable GPIO warnings
GPIO.setwarnings(False)

def setup_gpio():
    GPIO.cleanup()
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([MOTOR_1_PIN_1, MOTOR_1_PIN_2, MOTOR_2_PIN_1, MOTOR_2_PIN_2], GPIO.OUT)

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

def move_forward(speed):
    print("Moving forward")
    GPIO.output(MOTOR_1_PIN_1, True)
    GPIO.output(MOTOR_1_PIN_2, False)
    pwm_motor_2_pin_1.ChangeDutyCycle(speed)
    pwm_motor_2_pin_2.ChangeDutyCycle(0)

def turn_left(speed):
    print("Turning left")
    GPIO.output(MOTOR_1_PIN_1, False)
    GPIO.output(MOTOR_1_PIN_2, True)
    pwm_motor_2_pin_1.ChangeDutyCycle(speed)
    pwm_motor_2_pin_2.ChangeDutyCycle(0)

def turn_right(speed):
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

def calibrate_robot():
    global TURNING_TIME
    speed = DEFAULT_SPEED
    TURNING_TIME = DEFAULT_TURNING_TIME

    while True:
        print("Calibration Menu:")
        print("1. Calibrate Speed")
        print("2. Calibrate Turning Time")
        print("3. Exit Calibration")

        choice = input("Enter your choice (1-3): ")

        if choice == '1':
            speed = int(input("Enter new speed (0-100): "))
            print("Testing speed calibration...")
            move_forward(speed)
            time.sleep(15)
            stop_motors()
        elif choice == '2':
            TURNING_TIME = float(input("Enter new turning time (in seconds): "))
            print("Testing turning time calibration...")
            for _ in range(4):
                turn_left(speed)
                time.sleep(TURNING_TIME)
                stop_motors()
                time.sleep(2)
            for _ in range(4):
                turn_right(speed)
                time.sleep(TURNING_TIME)
                stop_motors()
                time.sleep(2)
        elif choice == '3':
            print("Exiting calibration.")
            break
        else:
            print("Invalid choice. Please try again.")

if __name__ == "__main__":
    setup_gpio()
    setup_pwm()
    try:
        calibrate_robot()
    except KeyboardInterrupt:
        pass
    finally:
        stop_motors()
        cleanup_gpio()
