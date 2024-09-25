import redis
import time
import vlc


REDIS_HOST = "10.36.166.15"
REDIS_PORT = "6379"


class MusicPlayer:

    def __init__(self):
        super().__init__()
        self.redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)
        self.players = {
            0: vlc.MediaPlayer("sounds/base.mp3"),
            1: vlc.MediaPlayer("sounds/arm_j0_torso.mp3"),
            2: vlc.MediaPlayer("sounds/arm_j1_shoulder.mp3"),
            3: vlc.MediaPlayer("sounds/arm_j3_elbow.mp3"),
            4: vlc.MediaPlayer("sounds/arm_j5_wrist.mp3"),
            5: vlc.MediaPlayer("sounds/arm_j6_hand.mp3"),
            6: vlc.MediaPlayer("sounds/gripper.mp3"),
        }

    def play(self, sound):
        p = self.players[sound]
        if p.get_state() != vlc.State.Playing:
            p.stop()
            p.play()

    def stop(self, sound):
        p = self.players[sound]
        if p.get_state() != vlc.State.Stopped:
            p.stop()

    def update(self):
        for i in self.players:
            play_str = self.redis_client.get(f"music::{i}")
            if play_str is None: continue

            play = play_str.decode("utf-8")
            if play == "play":
                self.play(i)
            else:
                self.stop(i)


if __name__ == "__main__":
    player = MusicPlayer()
    while True:
        player.update()
        time.sleep(0.1)
    