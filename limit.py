import time
import RPi.GPIO as GPIO

# Limit switch pin
LIMIT_SWITCH_PIN = 7

# Disable GPIO warnings
GPIO.setwarnings(False)

def limit_switch_callback(channel):
    print(f"Limit switch on pin {channel} pressed")

def setup_gpio():
    GPIO.cleanup()
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    try:
        GPIO.add_event_detect(LIMIT_SWITCH_PIN, GPIO.FALLING, callback=limit_switch_callback, bouncetime=200)
        print("GPIO setup complete")
    except RuntimeError as e:
        print(f"Failed to add edge detection: {e}")

def cleanup_gpio():
    GPIO.cleanup()
    print("GPIO cleaned up")

def main():
    setup_gpio()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup_gpio()

if __name__ == "__main__":
    main()
