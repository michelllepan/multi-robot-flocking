import argparse
from io import BytesIO

import cv2
import matplotlib.pyplot as plt
import numpy as np
import redis
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from PIL import Image

from flocking.utils import Goal, Pose, Humans

REDIS_HOST = "localhost"
# REDIS_HOST = "10.5.90.8"
REDIS_PORT = "6379"

GOAL_COLOR = "mediumpurple"
ROBOT_COLOR = "steelblue"
CARROT_COLOR = "coral"
HUMAN_COLOR = "seagreen"
HUMAN_CANDIDATE_COLOR = "darkseagreen"

class Visualizer:

    def __init__(self, robots):
        super().__init__()
        self.robots = robots
        self.num_robots = len(robots)
        self.redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)
        self.redis_keys = {}

        self.goals = {}
        self.carrots = {}
        self.poses = {}
        self.images = {}
        self.humans = {}
        self.filtered_human = None

        for r in self.robots:
            self.redis_keys[r] = {}
            self.redis_keys[r]["goal"] = "robot_" + str(r) + "::goal"
            self.redis_keys[r]["carrot"] = "robot_" + str(r) + "::carrot"
            self.redis_keys[r]["pose"] = "robot_" + str(r) + "::pose"
            self.redis_keys[r]["humans"] = "robot_" + str(r) + "::humans"
            self.redis_keys[r]["image"] = "robot_" + str(r) + "::color_image"

            self.goals[r] = Goal(x=-1.0, y=-1.0)
            self.poses[r] = Pose(x=-1.0, y=-1.0, h=0.0)

        self.filtered_human_key = "filtered_human"

    def read_redis(self):
        for r in self.robots:
            pose_string = self.redis_client.get(self.redis_keys[r]["pose"])
            pose = Pose.from_string(pose_string)
            if pose is not None:
                self.poses[r] = pose

            goal_string = self.redis_client.get(self.redis_keys[r]["goal"])
            goal = Goal.from_string(goal_string)
            if goal is not None:
                self.goals[r] = goal

            carrot_string = self.redis_client.get(self.redis_keys[r]["carrot"])
            carrot = Goal.from_string(carrot_string)
            self.carrots[r] = carrot

            human_string = self.redis_client.get(self.redis_keys[r]["humans"])
            humans = Humans.from_string(human_string)
            if humans is not None:
                self.humans[r] = humans

            image_data = self.redis_client.get(self.redis_keys[r]["image"])
            if image_data is None:
                image_array = np.zeros((640, 360, 3)).astype(np.uint8)
            else:
                image = Image.open(BytesIO(image_data))
                image_array = np.array(image)
                image_array = cv2.resize(image_array, (360, 640), interpolation=cv2.INTER_AREA)
            self.images[r] = image_array

        human_string = self.redis_client.get(self.filtered_human_key)
        if human_string:
            self.filtered_human = eval(human_string)

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

    @property
    def carrot_positions(self):
        return np.array([
            [self.carrots[r].x, self.carrots[r].y] if self.carrots[r] else [-1, -1]
            for r in self.robots ])
    
    @property
    def human_candidate_positions(self):
        to_concat = [self.humans[r].coords 
                     for r in self.robots if (r in self.humans and self.humans[r].coords)]
        if not to_concat: return None
        return np.concatenate([
            self.humans[r].coords
            for r in self.robots if self.humans[r].coords], axis=0)

    @property
    def filtered_human_position(self):
        if not self.filtered_human:
            return np.array([-1, -1])
        return np.array(self.filtered_human).flatten()

    def show_plot(self):
        while True:
            fig, ax = plt.subplots(figsize=(10,5))
            self.read_redis()

            for i in range(self.num_robots):
                r = self.robots[i]
                ax.scatter(
                    x=self.robot_positions[i][0],
                    y=self.robot_positions[i][1],
                    c=ROBOT_COLOR,
                    marker=f"${r}$")
                ax.scatter(
                    x=self.goal_positions[i][0],
                    y=self.goal_positions[i][1],
                    c=GOAL_COLOR,
                    marker=f"${r}$")
                ax.scatter(
                    x=self.carrot_positions[i][0],
                    y=self.carrot_positions[i][1],
                    c=CARROT_COLOR,
                    marker=f"${r}$")
                
            if self.human_candidate_positions is not None:
                ax.scatter(
                    x=self.human_candidate_positions[:,0],
                    y=self.human_candidate_positions[:,1],
                    c=HUMAN_CANDIDATE_COLOR,
                    marker="$h$")

                ax.scatter(
                    x=self.filtered_human_position[0],
                    y=self.filtered_human_position[1],
                    c=HUMAN_COLOR,
                    marker="$H$")
            
            ax.set_xlim(-1, 10)
            ax.set_ylim(-1, 5)

            robot_patch = Line2D([0], [0], marker="o", color=ROBOT_COLOR, linestyle="None", label="robot")
            goal_patch = Line2D([0], [0], marker="o", color=GOAL_COLOR, linestyle="None", label="target")
            
            ax.legend(handles=[robot_patch, goal_patch])

            fig.canvas.draw()
            fig_array = np.array(fig.canvas.renderer._renderer)
            fig_array = cv2.cvtColor(fig_array, cv2.COLOR_RGB2BGR)
            fig_array = cv2.resize(fig_array, (1280, 640), interpolation=cv2.INTER_AREA)
            plt.close(fig)

            images = [self.images[r] for r in self.robots]
            all_images = np.concatenate([fig_array] + images, axis=1)

            cv2.imshow("figure", all_images)
            if cv2.waitKey(1) & 0xFF == ord('q'): 
                break


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", "-r", nargs="+", type=int, required=True)
    args = parser.parse_args()

    visualizer = Visualizer(robots=tuple(args.robots))
    visualizer.show_plot()
