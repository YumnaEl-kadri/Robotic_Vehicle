import sys
import traceback
import RPi.GPIO as GPIO
from pathlib import Path
import pathlib
import cv2

temp = pathlib.WindowsPath
pathlib.WindowsPath = pathlib.PosixPath
import torch
import time
import threading
from picamera2 import Picamera2
from runningMotor import *
from ultrasonic_sensor import *

GPIO.setmode(GPIO.BCM)

POWER = 25
tiny_short_sleep = 0.2
short_sleep = 0.5
sleep = 1


class CarDetection:
    def __init__(self, camera):
        self.camera = camera
        self.model = torch.hub.load(
            "/home/robotic-vehicle/Desktop/objectDetection/yolov5-master",
            "custom",
            path="/home/robotic-vehicle/Desktop/objectDetection/yolov5-master/runs/train/exp2/weights/last.pt",
            source="local",
            force_reload=True,
        )
        self.x_goal = None
        self.x_goal_width = None 
        self.last_known_x_goal = None  
        self.lock = threading.Lock()
        self.continue_running = True
        self.count = 0

    def detect_ball(self, frame):
        result = self.model(frame)
        result.render()
        xyxy = result.xyxy[0]
        detection = None
        detection_width = None
        for det in xyxy:
            x1, y1, x2, y2, confidence, cls = det
            if confidence.item() > 0.7: 
                detection_width = x2.item() - x1.item()
                center_x = (x1.item() + x2.item()) / 2
                detection = center_x
        return detection, detection_width

    def run(self):
        while self.continue_running:
            start = time.time()

            frame = self.camera.capture_array()

            if frame is None:
                continue

            flipped_frame = cv2.flip(frame, -1)

            detection, detection_width = self.detect_ball(flipped_frame)
            end = time.time()
            if detection is not None:
                #print(f"after detect_ball {end - start}")

                self.last_known_x_goal = self.x_goal
                self.x_goal = detection
                self.x_goal_width = detection_width
                self.count = 0

                if self.x_goal_width >= 130:
                    stop()
                    time.sleep(0.1)
                    turn_vehicle_to_direction(self.x_goal)
                    time.sleep(6)
                    stop()
                    time.sleep(0.1)
                    close_program()
                else:
                    adjust_direction(POWER, self.x_goal)
            else:
                stop()
                time.sleep(tiny_short_sleep)
                rotate_vehicle(self.last_known_x_goal, self.count)
                self.count += 1
                self.x_goal = None
                self.x_goal_width = None

    def stop(self):
        self.continue_running = False

    def get_X(self):
        return self.x_goal

    def get_X_width(self):
        return self.x_goal_width

    def get_known_x_goal(self):
        if self.x_goal != None:
            return self.x_goal
        else:
            return self.last_known_x_goal

    def get_last_known_x_goal(self):
        return self.last_known_x_goal


def adjust_direction(POWER, x_goal):

    if x_goal == None:
        #print("x_goal is None, stopping the motor.")
        stop()
        return

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
        stop()


def rotate_vehicle(x, count):
    if x != None:
        stop()
        time.sleep(short_sleep)
        turn_vehicle_to_direction(x)
        time.sleep(short_sleep)
        stop()
        time.sleep(2)
        if count == 12:
            close_program()


def turn_vehicle_to_direction(x):
    if x == None:
        return None
    elif x > 320:
        turn_right(POWER)
    else:
        turn_left(POWER)


def log_message(message):
    if can_log:
        print(message)


def close_program():
    detector.stop()
    
if __name__ == "__main__":
    global can_log

    piCam = Picamera2()
    preview_config = piCam.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )

    piCam.configure(preview_config)
    piCam.start()

    can_log = False
    try:
        detector = CarDetection(piCam)
        detector.run()
    except Exception as ex:
        print("Exception happened.")
        print(ex)
        traceback.print_exc()
    finally:
        stop()
        detector.stop()
        piCam.stop()
        GPIO.cleanup()
        cv2.destroyAllWindows()
        print("System shutdown cleanly")
        sys.exit()