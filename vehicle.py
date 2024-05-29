import sys
import traceback
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import time
import threading
from ObjectDetector import *
from runningMotor import *
from ultrasonic_sensor import *
from enum import Enum

GPIO.setmode(GPIO.BCM)

POWER = 25
tiny_short_sleep = 0.2
short_sleep = 0.5
sleep = 1

# source env/bin/activate
# cd Desktop/test_sensors/motor-test/
# python3 vehicle.py


def close_program():
    global is_target_reached, stop_threads
    stop_threads = True
    is_target_reached = True


class LogType(Enum):
    Information = 1
    Important = 2


def detect_ball():
    global ball_lost, x_val, last_x_val, known_x, get_X_width, counter

    log_message("detect ball started..", LogType.Important)
    time.sleep(0.01)

    detector.run()
    x_val = detector.get_X()
    last_x_val = detector.get_last_known_x_goal()
    known_x = detector.get_known_x_goal()
    get_X_width = detector.get_X_width()
    counter = detector.get_count()

    log_message(f"x_val = {x_val}", LogType.Important)
    ball_lost = x_val == None
    log_message(f"ball_lost = {ball_lost}", LogType.Important)

    if not ball_lost:
        log_message(f"ball detected x={x_val}")


def detect_obstacle():
    global is_obstacle_reached
    log_message("detect obstacle started..")

    while not is_target_reached and not stop_threads:
        time.sleep(0.01)
        is_obstacle_reached = obstacle_reached()
        if is_obstacle_reached:
            stop()
            time.sleep(short_sleep)
            avoid_obstacle()


def avoid_obstacle():
    if x_val is not None:
        go_back(x_val)


def adjust_direction(POWER, x_goal):
    if get_X_width is not None:
        if get_X_width >= 130:
            stop()
            time.sleep(0.1)
            turn_vehicle_to_direction(x_goal)
            time.sleep(6.5)
            stop()
            time.sleep(0.1)
            close_program()
    if x_goal == None:
        #print("x_goal is None, stopping the motor.")
        stop()
        return
    # print(x_goal)
    if 200 <= x_goal < 300:
        turn_forward(POWER, bias=-5)
    elif 100 <= x_goal < 200:
        turn_forward(POWER, bias=-10)
    elif 0 <= x_goal < 100:
        turn_forward(POWER, bias=-15)

    elif 340 < x_goal <= 440:
        turn_forward(POWER, bias=5)
    elif 440 < x_goal <= 540:
        turn_forward(POWER, bias=10)
    elif 540 < x_goal <= 640:
        turn_forward(POWER, bias=15)

    elif 300 <= x_goal <= 340:
        forward(POWER)
    else:
        print("stop")
        stop()


def go_back(x):
    global x_val, ball_lost, is_obstacle_reached
    # print("Going back...")
    adjust_direction(-POWER, x)
    time.sleep(2)
    stop()
    time.sleep(tiny_short_sleep)

    rotate_vehicle(last_x_val, tiny_short_sleep, counter)

    x_val = None
    ball_lost = True
    detect_ball()
    time.sleep(sleep)
    stop()
    time.sleep(sleep)
    while stop_threads and x_val is None:
        time.sleep(tiny_short_sleep)
        rotate_vehicle(last_x_val, short_sleep, counter)
        detect_ball()


def rotate_vehicle(x, sleep_for_turn, counter):
    if x != None:
        log_message(f"Rotating the vehicle... x = {x}", LogType.Information)
        stop()
        time.sleep(short_sleep)
        turn_vehicle_to_direction(x)
        time.sleep(sleep_for_turn)
        stop()
        time.sleep(4)
        # print(counter)
        if counter == 12:
            close_program()


def turn_vehicle_to_direction(x):
    if x == None:
        return None
    elif x > 320:
        turn_right(POWER)
    else:
        turn_left(POWER)


def log_message(message, log_type=LogType.Important):
    if can_log and log_type == LogType.Important:
        print(message)


if __name__ == "__main__":
    global can_log, stop_threads, is_target_reached, is_obstacle_reached, ball_lost, x_val, last_x_val, capture_images_thread, known_x, get_X_width, counter

    can_log = False

    stop_threads = False
    is_target_reached = False
    is_obstacle_reached = False
    ball_lost = True

    x_val = None
    last_x_val = None
    known_x = None
    get_X_width = None
    counter = 0

    piCam = Picamera2()
    preview_config = piCam.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )

    piCam.configure(preview_config)
    piCam.start()

    detector = ObjectDetector(piCam)

    log_message("io started.")
    obstacle_thread = threading.Thread(target=detect_obstacle)
    obstacle_thread.start()

    distance_thread = threading.Thread(target=distance_to_obstacle)
    distance_thread.start()

    try:
        while not is_target_reached and not stop_threads:
            # detect_ball()
            while not is_obstacle_reached and not ball_lost and not stop_threads:
                # time.sleep(0.01)
                detect_ball()
                log_message("ball detected", LogType.Important)
                log_message("go go go", LogType.Important)
                adjust_direction(POWER, x_val)
            while is_obstacle_reached and not stop_threads:
                # time.sleep(0.01)
                stop()
                time.sleep(short_sleep)
                detect_ball()
            while ball_lost and not stop_threads and x_val is None:
                # log_message("rotate")
                # time.sleep(0.1)
                rotate_vehicle(last_x_val, short_sleep, counter)
                detect_ball()

    except Exception as ex:
        # print("Exception happened.")
        traceback.print_exc()
    finally:
        stop_threads = True
        is_target_reached = True

        stop_ultrasonic()
        stop()
        detector.stop()
        piCam.stop()

        distance_thread.join()
        obstacle_thread.join()
        can_log = False
        GPIO.cleanup()
        cv2.destroyAllWindows()
        print("System shutdown cleanly")
        sys.exit()
