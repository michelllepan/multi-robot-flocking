import argparse
from functools import partial

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import redis
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

from flocking.utils import Goal, Pose

REDIS_HOST = "localhost"
REDIS_PORT = "6379"

GOAL_COLOR = "mediumpurple"
ROBOT_COLOR = "steelblue"


class Visualizer:

    def __init__(self, robots):
        super().__init__()
        self.robots = robots
        self.num_robots = len(robots)
        self.redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)

        self.goal_keys = {}
        self.pose_keys = {}

        self.goals = {}
        self.poses = {}

        for r in self.robots:
            self.goal_keys[r] = "robot_" + str(r) + "::goal"
            self.pose_keys[r] = "robot_" + str(r) + "::pose"

            self.goals[r] = Goal(x=-1.0, y=-1.0)
            self.poses[r] = Pose(x=-1.0, y=-1.0, h=0.0)

    def read_redis(self):
        for r in self.robots:
            pose_string = self.redis_client.get(self.pose_keys[r])
            pose = Pose.from_string(pose_string)
            if pose is not None:
                self.poses[r] = pose

            goal_string = self.redis_client.get(self.goal_keys[r])
            goal = Goal.from_string(goal_string)
            if goal is not None:
                self.goals[r] = goal

    @property
    def robot_positions(self):
        return np.array([
            [self.poses[r].x, self.poses[r].y]
            for r in self.robots ])

    @property
    def goal_positions(self):
        return np.array([
            [self.goals[r].x, self.goals[r].y]
            for r in self.robots ])

    def show_plot(self):
        fig, ax = plt.subplots()

        scats = []
        for i in range(self.num_robots):
            r = self.robots[i]
            scats.append(ax.scatter(
                x=self.robot_positions[i][0],
                y=self.robot_positions[i][1],
                c=ROBOT_COLOR,
                marker=f"${r}$"))
            scats.append(ax.scatter(
                x=self.goal_positions[i][0],
                y=self.goal_positions[i][1],
                c=GOAL_COLOR,
                marker=f"${r}$"))
        
        ax.set_xlim(-1, 16)
        ax.set_ylim(-1, 13)

        robot_patch = Line2D([0], [0], marker="o", color=ROBOT_COLOR, linestyle="None", label="robot")
        goal_patch = Line2D([0], [0], marker="o", color=GOAL_COLOR, linestyle="None", label="target")
        
        ax.legend(handles=[robot_patch, goal_patch])

        def update(frame, scats, visualizer):
            visualizer.read_redis()
            for i in range(visualizer.num_robots):
                scats[i * 2].set_offsets(visualizer.robot_positions[i])
                scats[i * 2 + 1].set_offsets(visualizer.goal_positions[i])
            return scats

        ani = animation.FuncAnimation(
            fig=fig,
            func=partial(update, scats=scats, visualizer=self),
            frames=200,
            interval=50,
            blit=True,
        )
        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", "-r", nargs="+", type=int, required=True)
    args = parser.parse_args()

    visualizer = Visualizer(robots=tuple(args.robots))
    visualizer.show_plot()