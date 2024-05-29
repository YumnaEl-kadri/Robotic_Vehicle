import RPi.GPIO as GPIO

def setup_gpio():
    if not hasattr(GPIO, 'mode_set') or not GPIO.mode_set:
        GPIO.setmode(GPIO.BCM)
        GPIO.mode_set = True  # Custom attribute to ensure we don't set the mode multiple times

def cleanup_gpio():
    GPIO.cleanup()
