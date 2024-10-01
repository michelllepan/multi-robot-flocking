import time

from flocking.robot import MusicPlayer


player = MusicPlayer()
while True:
    player.update()
    time.sleep(0.1)
