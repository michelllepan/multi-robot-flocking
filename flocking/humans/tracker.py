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
from .detector import MediaPipeDetector


REDIS_POS_KEY = "sai2::realsense::"
STREAMING_POINTS = ["center_shoulder"]
EMA_BETA = 0.9

class HumanTracker:

    def __init__(self, show_frames: bool = False):
        self.camera = RealSenseCamera()
        self.detector = MediaPipeDetector()
        self.show_frames = show_frames
        
        # # initialize history
        # self.history = {}
        # self.history_length = history_length
        # for key in STREAMING_POINTS:
        #     self.history[key] = np.empty((history_length, 3))
        #     self.history[key][:] = np.nan

    # def smooth_values(self, key, new_value):
    #     # update history
    #     self.history[key] = np.concatenate((
    #         self.history[key][1:],
    #         np.array(new_value).reshape((1, 3))
    #     ))

    #     # create weights
    #     weights = (1 - EMA_BETA) * np.power(EMA_BETA, np.arange(self.history_length))

    #     # redistribute weights if values are nan
    #     for i in range(self.history_length):
    #         if np.isnan(self.history[key][i]).any():
    #             weights[i] = 0
    #     if sum(weights) == 0:
    #         return None
    #     weights = weights / sum(weights)

    #     # calculate smoothed values
    #     smoothed_values = np.dot(weights.T, np.nan_to_num(self.history[key]))
    #     return smoothed_values

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
        height, width, _ = depth_colormap_dim
        color_colormap_dim = color_image.shape

        if depth_colormap_dim != color_colormap_dim:
            color_image = cv2.resize(
                color_image, 
                dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                interpolation=cv2.INTER_AREA)

        detection_result = self.detector.run_detection(color_image)
        color_image = self.detector.draw_landmarks_on_image(color_image, detection_result)
        depth_colormap = self.detector.draw_landmarks_on_image(depth_colormap, detection_result)
        images = np.hstack((color_image, depth_colormap))

        if self.show_frames:
            cv2.imshow("RealSense", images)

        landmark_dicts = self.detector.parse_landmarks(detection_result)

        def get_depth_at_pixel(x, y):
            x, y = int(x * width), int(y * height)
            if x < 0 or x >= width or y < 0 or y >= height:
                return None
            else:
                return 1e-3 * depth_image[y, x]

        people = []
        for landmark_dict in landmark_dicts:
            coords = landmark_dict["center_shoulder"]
            depth = get_depth_at_pixel(coords[0], coords[1])
            if depth is None:
                continue

            x = depth
            y = width * (0.5 - coords[0]) * (depth / 640)
            people.append([x, y])

            # print(f"x: {x : 5.2f}   y: {y : 5.2f}")

        return people


if __name__ == "__main__":
    tracker = HumanTracker(show_frames=True)
    while True:
        tracker.process_frame()
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break