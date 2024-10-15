import os
import time

from flocking.robot import MusicPlayer



def main(robot_id=1, redis_host="localhost", redis_port="6379"):
    player = MusicPlayer(robot_id, redis_host, redis_port)
    while True:
        player.update()
        time.sleep(0.1)

if __name__ == '__main__':
    robot_id = os.environ.get("ROBOT_ID", 1)
    redis_host = os.environ.get("REDIS_HOST", "localhost")
    redis_port = os.environ.get("REDIS_PORT", "6379")
    main(robot_id=robot_id, redis_host=redis_host, redis_port=redis_port)
