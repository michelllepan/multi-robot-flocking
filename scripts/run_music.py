import os
import time

from flocking.robot import MusicPlayer



def main(robot_id=1):
    player = MusicPlayer(robot_id=robot_id)
    while True:
        player.update()
        time.sleep(0.1)

if __name__ == '__main__':
    robot_id = os.environ.get("ROBOT_ID", 1)
    main(robot_id=robot_id)
