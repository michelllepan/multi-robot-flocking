import argparse
import time
from multiprocessing import Process

from flocking.director import ArmMover, FlockPlanner, Visualizer


def run_planner(robots):
    director = FlockPlanner(robots=robots)
    arm_mover = ArmMover(robots=robots)
    while True:
        director.step_flocking()
        arm_mover.send_arm_commands()
        time.sleep(0.1) 

def run_visualizer(robots):
    visualizer = Visualizer(robots=robots)
    visualizer.show_plot()

def main(robots=(1,)):
    p1 = Process(target=run_planner, args=(robots,))
    p1.start()

    p2 = Process(target=run_visualizer, args=(robots,))
    p2.start()

    p1.join()
    p2.join()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", "-r", nargs="+", type=int, required=True)
    args = parser.parse_args()
    main(robots=tuple(args.robots))
