import time
import RPi.GPIO as GPIO

# Define the pin number for the button
TURN_ON_OFF_BUTTON_PIN = 23

# Disable GPIO warnings
GPIO.setwarnings(False)

def setup_gpio():
    GPIO.cleanup()
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(TURN_ON_OFF_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def cleanup_gpio():
    GPIO.cleanup()
    print("GPIO cleaned up")

def test_button():
    setup_gpio()
    try:
        while True:
            print(GPIO.input(TURN_ON_OFF_BUTTON_PIN))
            time.sleep(0.1)  # Check button state every 100ms
    except KeyboardInterrupt:
        cleanup_gpio()

if __name__ == "__main__":
    test_button()
