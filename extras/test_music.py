import vlc
import time

p = vlc.MediaPlayer("sounds/base.mp3")
p.play()

while True:
    state = p.get_state()
    time.sleep(0.1)