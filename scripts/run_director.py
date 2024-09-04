import argparse
import time
from multiprocessing import Process

from flocking.director import FlockPlanner, Visualizer


def main(robots=(1,)):

    def run_planner():
        director = FlockPlanner(robots=robots)
        while True:
            director.step_flocking()
            time.sleep(0.1) 

    def run_visualizer():
        visualizer = Visualizer(robots=robots)
        visualizer.show_plot()
        
    p1 = Process(target=run_planner)
    p1.start()

    p2 = Process(target=run_visualizer)
    p2.start()

    p1.join()
    p2.join()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", "-r", nargs="+", type=int, required=True)
    args = parser.parse_args()
    main(robots=tuple(args.robots))
