import RPi.GPIO as GPIO
import time
import threading


# Global variable for distance measurement
current_distance = -1
distance_lock = threading.Lock()
continue_running = True


TRIG_PIN = 5
ECHO_PIN = 6

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)


def distance_to_obstacle():
    global current_distance

    try:
        while continue_running:
            # continue_running = False
            GPIO.output(TRIG_PIN, True)
            time.sleep(0.0001)  # Trigger pulse width of 100 microseconds
            GPIO.output(TRIG_PIN, False)

            pulse_start_time = time.time()
            timeout_start = pulse_start_time

            # Wait for the pulse to start
            while GPIO.input(ECHO_PIN) == 0:
                pulse_start = time.time()
                if pulse_start - timeout_start > 0.02:  # 20 ms timeout for start
                    pulse_start = None
                    break

            if pulse_start is None:
                continue  # Skip this cycle if no pulse start was detected

            timeout_start = time.time()
            # Wait for the pulse to stop
            while GPIO.input(ECHO_PIN) == 1:
                pulse_end = time.time()
                if pulse_end - timeout_start > 0.02:  # 20 ms timeout for end
                    pulse_end = None
                    break

            if pulse_end is None:
                continue  # Skip this cycle if no pulse end was detected

            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Calculate distance in centimeters

            with distance_lock:
                current_distance = round(distance, 2)


    except KeyboardInterrupt:
        GPIO.cleanup()
    except Exception as e:
        print(f"Error in distance measurement thread: {e}")
        GPIO.cleanup()  # Ensure GPIO is cleaned up on error


def get_current_distance():
    return current_distance


def obstacle_reached():
    distance = get_current_distance()
    return distance is not None and 0 <= distance <= 15

def stop_ultrasonic():
    global continue_running
    continue_running = False
