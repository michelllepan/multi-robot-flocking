import argparse
import time
from multiprocessing import Process

from flocking.director import CameraFeed, FlockPlanner, Visualizer


def run_planner(robots):
    director = FlockPlanner(robots=robots)
    while True:
        director.step_flocking()
        time.sleep(0.1) 

def run_visualizer(robots):
    visualizer = Visualizer(robots=robots)
    visualizer.show_plot()

def run_feed(robots):
    feed = CameraFeed(robots=robots)
    feed.display_feed()

def main(robots=(1,)):
    p1 = Process(target=run_planner, args=(robots,))
    p1.start()

    p2 = Process(target=run_visualizer, args=(robots,))
    p2.start()

    p3 = Process(target=run_feed, args=(robots,))
    p3.start()

    p1.join()
    p2.join()
    p3.join()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", "-r", nargs="+", type=int, required=True)
    args = parser.parse_args()
    main(robots=tuple(args.robots))
