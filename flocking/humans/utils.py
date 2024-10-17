import numpy as np

def get_depth_at_pixel(depth_image, x, y):
    height, width = depth_image.shape
    x, y = int(x * width), int(y * height)
    if x < 0 or x >= width or y < 0 or y >= height:
        return None
    else:
        return 1e-3 * depth_image[y, x]

def parse_landmarks(detection_result):
    if not detection_result.pose_landmarks:
        return []
    
    pose_landmarks = detection_result.pose_landmarks[0]

    def landmark_to_vec(landmark):
        return np.array([landmark.x, landmark.y, landmark.z])

    landmarks = []
    for pose_landmarks in detection_result.pose_landmarks:

        # LEFT
        left_pinky = landmark_to_vec(pose_landmarks[17])
        left_index = landmark_to_vec(pose_landmarks[19])
        left_thumb = landmark_to_vec(pose_landmarks[21])
        left_hand = np.mean((left_pinky, left_index, left_thumb), axis=0)

        left_elbow = landmark_to_vec(pose_landmarks[13])
        left_shoulder = landmark_to_vec(pose_landmarks[11])
        left_hip = landmark_to_vec(pose_landmarks[23])

        # RIGHT
        right_pinky = landmark_to_vec(pose_landmarks[18])
        right_index = landmark_to_vec(pose_landmarks[20])
        right_thumb = landmark_to_vec(pose_landmarks[22])
        right_hand = np.mean((right_pinky, right_index, right_thumb), axis=0)

        right_elbow = landmark_to_vec(pose_landmarks[14])
        right_shoulder = landmark_to_vec(pose_landmarks[12])
        right_hip = landmark_to_vec(pose_landmarks[24])

        # CENTER
        center_shoulder = np.mean((left_shoulder, right_shoulder), axis=0)
        center_hips = np.mean((left_hip, right_hip), axis=0)

        landmarks.append({
            "left_hand": left_hand,
            "left_elbow": left_elbow,
            "left_shoulder": left_shoulder,
            "right_hand": right_hand,
            "right_elbow": right_elbow,
            "right_shoulder": right_shoulder,
            "center_shoulder": center_shoulder,
            "center_hips": center_hips})

    return landmarks