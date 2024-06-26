import numpy as np


def clip_by_norm(
    vector: np.ndarray,
    max_magnitude: float,
) -> np.ndarray:
    magnitude = np.linalg.norm(vector)
    if magnitude > max_magnitude:
        unit_vec = vector / magnitude
        return unit_vec * max_magnitude
    else:
        return vector


def normalize(
    vector: np.ndarray,
) -> np.ndarray:
    magnitude = np.linalg.norm(vector)
    if magnitude < 1e-6:
        return np.zeros_like(vector)
    else:
        return vector / magnitude