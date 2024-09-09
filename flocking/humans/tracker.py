import argparse
import os
import time
from datetime import datetime
from typing import Sequence

import cv2
import numpy as np
import redis
import scipy

from .camera import RealSenseCamera
from .detector import MediaPipeDetector, draw_landmarks_on_image, parse_landmarks


REDIS_POS_KEY = "sai2::realsense::"
STREAMING_POINTS = ["center_shoulder"]
EMA_BETA = 0.9


def get_depth_at_pixel(depth_image, x, y):
    height, width = depth_image.shape
    x, y = int(x * width), int(y * height)
    if x < 0 or x >= width or y < 0 or y >= height:
        return None
    else:
        return 1e-3 * depth_image[y, x]

class HumanTracker:

    def __init__(self, human_callback, show_frames: bool = False):
        self.camera = RealSenseCamera()
        self.detector = MediaPipeDetector(result_callback=self.detector_callback)

        self.human_callback = human_callback
        self.show_frames = show_frames

        self.image_history = {}
        
    def process_frame(self):
        depth_frame, color_frame = self.camera.get_frames()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # rotate images
        depth_image = np.rot90(depth_image, 3)
        color_image = np.rot90(color_image, 3)

        depth_image = scipy.signal.convolve2d(
            in1=depth_image,
            in2=np.ones((3, 3)) / 9,
            mode="same",
        )

        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),cv2.COLORMAP_JET)
        
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        if depth_colormap_dim != color_colormap_dim:
            color_image = cv2.resize(
                color_image, 
                dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                interpolation=cv2.INTER_AREA)

        timestamp = int(1000 * time.time())
        self.detector.run_detection(color_image, timestamp)
        self.image_history[timestamp] = (color_image, depth_image)
        
    def detector_callback(self, detection_result, output_image, timestamp):
        if timestamp not in self.image_history: 
            return

        color_image, depth_image = self.image_history.pop(timestamp)
        height, width = depth_image.shape
        # depth_colormap = cv2.applyColorMap(
        #     cv2.convertScaleAbs(depth_image, alpha=0.03),cv2.COLORMAP_JET)

        color_image = draw_landmarks_on_image(color_image, detection_result)
        # depth_colormap = draw_landmarks_on_image(depth_colormap, detection_result)
        # images = np.hstack((color_image, depth_colormap))

        # if self.show_frames:
        #     cv2.imshow("RealSense", images)

        landmark_dicts = parse_landmarks(detection_result)

        humans = []
        for landmark_dict in landmark_dicts:
            coords = landmark_dict["center_shoulder"]
            depth = get_depth_at_pixel(depth_image, coords[0], coords[1])
            if depth is None:
                continue

            x = depth
            y = width * (0.5 - coords[0]) * (depth / 640)
            humans.append([x, y])

        self.human_callback(humans, color_image)
        self.image_history = {k: v for k, v in self.image_history.items() if k > timestamp}


if __name__ == "__main__":
    tracker = HumanTracker(show_frames=True)
    while True:
        tracker.process_frame()
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break