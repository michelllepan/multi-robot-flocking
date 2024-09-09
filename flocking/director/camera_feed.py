import argparse
import time
from io import BytesIO

import cv2
import numpy as np
import redis
from PIL import Image

REDIS_HOST = "localhost"
REDIS_PORT = "6379"

IMAGE_SIZE = (360, 640)

class CameraFeed:

    def __init__(self, robots):
        super().__init__()
        self.robots = robots
        self.redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)
        
        self.image_keys = {}
        for r in self.robots:
            self.image_keys[r] = "robot_" + str(r) + "::image"

    def get_image(self, robot):
        image_data = self.redis_client.get(self.image_keys[robot])
        image = Image.open(BytesIO(image_data))
        image_array = np.array(image)
        image_array = image_array[:,:,::-1]
        return image_array
                
    def display_feed(self):
        while True:
            images = [self.get_image(r) for r in self.robots]
            cv2.imshow("image", np.concatenate(images, axis=1))
            if cv2.waitKey(1) & 0xFF == ord('q'): 
                break


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", "-r", nargs="+", type=int, required=True)
    args = parser.parse_args()

    feed = CameraFeed(robots=tuple(args.robots))
    feed.display_feed()

