import cv2
import numpy as np
import pyrealsense2 as rs
import scipy

def main():

    width, height = 640, 360

    pipeline = rs.pipeline()
    config = rs.config()

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    advanced_mode = rs.rs400_advanced_mode(device)
    advanced_mode.toggle_advanced_mode(True)

    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
    pipeline.start(config)


if __name__ == "__main__":
    main()