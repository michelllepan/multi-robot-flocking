import argparse
import time

from flocking.director import FlockPlanner


def main(args=None, robots=(1,)):
    director = FlockPlanner(robots=robots)
    while True:
        director.step_flocking()
        time.sleep(0.1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", "-r", nargs="+", type=int, required=True)
    args = parser.parse_args()
    main(robots=tuple(args.robots))
