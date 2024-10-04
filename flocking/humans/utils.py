def get_depth_at_pixel(depth_image, x, y):
    height, width = depth_image.shape
    x, y = int(x * width), int(y * height)
    if x < 0 or x >= width or y < 0 or y >= height:
        return None
    else:
        return 1e-3 * depth_image[y, x]
