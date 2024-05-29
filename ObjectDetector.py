from pathlib import Path
import pathlib
import cv2

temp = pathlib.WindowsPath
pathlib.WindowsPath = pathlib.PosixPath
import torch
import time
import threading
from picamera2 import Picamera2


class ObjectDetector:
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
        # while self.continue_running:
        time.sleep(0.5)
        start = time.time()

        frame = self.camera.capture_array()

        if frame is None:
            return
            # continue

        flipped_frame = cv2.flip(
            frame, -1
        )  

        detection, detection_width = self.detect_ball(flipped_frame)
        end = time.time()
        with self.lock:
            if detection is not None:
                #print(f"after detect_ball {end - start}")

                self.last_known_x_goal = self.x_goal
                self.x_goal = detection
                self.x_goal_width = detection_width
                self.count = 0
            else:
                self.count +=1
                self.x_goal = None
                self.x_goal_width = None

    def stop(self):
        self.continue_running = False

    def get_X(self):
        return self.x_goal

    def get_X_width(self):
        return self.x_goal_width
    
    def get_count(self):
        return self.count

    def get_known_x_goal(self):
        if self.x_goal != None:
            return self.x_goal
        else:
            return self.last_known_x_goal

    def get_last_known_x_goal(self):
        return self.last_known_x_goal


# if __name__ == "__main__":

#     piCam = Picamera2()
#     preview_config = piCam.create_preview_configuration(
#         main={"size": (640, 480), "format": "RGB888"}
#     )

#     piCam.configure(preview_config)
#     piCam.start()

#     detector = ObjectDetector(piCam)
#     detector.run()
