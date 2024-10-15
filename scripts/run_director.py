import argparse
import time
from multiprocessing import Process

from flocking.director import ArmMover, FlockPlanner, Visualizer

REDIS_CONFIG = {
    0: ("localhost", "6379"), # director
    1: ("192.168.1.151", "6379"),
    2: ("192.168.1.152", "6379"),
    3: ("192.168.1.153", "6379"),
    4: ("192.168.1.154", "6379"),
}

def run_planner(robots, redis_config):
    director = FlockPlanner(robots=robots, redis_config=redis_config)
    arm_mover = ArmMover(robots=robots, redis_config=redis_config)
    while True:
        director.step_flocking()
        arm_mover.send_arm_commands()
        time.sleep(0.02) 

def run_visualizer(robots, redis_config):
    visualizer = Visualizer(robots=robots, redis_config=redis_config)
    visualizer.show_plot()

def main(robots=(1,)):
    redis_config = {i: REDIS_CONFIG[i] for i in [0] + list(robots)}
    p1 = Process(target=run_planner, args=(robots, redis_config))
    p1.start()

    p2 = Process(target=run_visualizer, args=(robots, redis_config))
    p2.start()

    p1.join()
    p2.join()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", "-r", nargs="+", type=int, required=True)
    args = parser.parse_args()
    main(robots=tuple(args.robots))
