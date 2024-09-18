class Pose:

    def __init__(self, x: float, y: float, h: float):
        super().__init__()
        self.x = x
        self.y = y
        self.h = h

    def __str__(self):
        return str([self.x, self.y, self.h])
    
    @classmethod
    def from_string(cls, pose_string: str):
        if pose_string is None: return None
        pose_list = eval(pose_string)
        if pose_list is None: return None
        return cls(x=pose_list[0], y=pose_list[1], h=pose_list[2])


class Goal:

    def __init__(self, x: float, y: float):
        super().__init__()
        self.x = x
        self.y = y

    def __str__(self):
        return str([self.x, self.y])

    @classmethod
    def from_string(cls, goal_string: str):
        if goal_string is None: return None
        goal_list = eval(goal_string)
        if goal_list is None: return None
        return cls(x=goal_list[0], y=goal_list[1])


class Humans:

    def __init__(self, coords: list):
        super().__init__()
        self.coords = coords

    def __str__(self):
        return str(self.coords)

    @classmethod
    def from_string(cls, human_string: str):
        if human_string is None: return None
        human_list = eval(human_string)
        if human_list is None: return None
        return cls(coords=human_list)