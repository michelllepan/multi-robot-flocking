import os
import time

from flocking.robot import MusicPlayer



def main(robot_id=1, redis_host="localhost"):
    player = MusicPlayer(robot_id, redis_host)
    while True:
        player.update()
        time.sleep(0.1)

if __name__ == '__main__':
    robot_id = os.environ.get("ROBOT_ID", 1)
    redis_host = os.environ.get("REDIS_HOST", "localhost")
    main(robot_id=robot_id, redis_host=redis_host)
