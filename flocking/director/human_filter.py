import numpy as np


class HumanFilter:

    def __init__(self,
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
    ):
        super().__init__()
        self.bounds_filter = lambda p: (p[0] > x_min and p[0] < x_max and
                                        p[1] > y_min and p[1] < y_max)

        x_center = (x_min + x_max) / 2
        y_center = (y_min + y_max) / 2
        self.dist_to_center = lambda p: np.linalg.norm([p[0] - x_center, 
                                                        p[1] - y_center])

    def apply(self, candidates: list, num: int):
        print("apply: before filter", candidates)
        in_bounds = filter(self.bounds_filter, candidates)
        in_bounds = list(in_bounds)
        print("apply: after filter", in_bounds)

        # temporary: choose human closest to center
        if len(in_bounds) <= 1:
            return in_bounds
        return min(in_bounds, key=self.dist_to_center)

