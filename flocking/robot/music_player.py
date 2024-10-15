import redis
import time
import vlc


class MusicPlayer:

    def __init__(self, robot_id, redis_host, redis_port):
        super().__init__()
        robot_name = "robot_" + str(robot_id)

        self.redis_client = redis.Redis(host=redis_host, port=redis_port)
        self.players = {
            0: vlc.MediaPlayer("sounds/base.mp3"),
            1: vlc.MediaPlayer("sounds/arm_j0_torso.mp3"),
            2: vlc.MediaPlayer("sounds/arm_j1_shoulder.mp3"),
            3: vlc.MediaPlayer("sounds/arm_j3_elbow.mp3"),
            4: vlc.MediaPlayer("sounds/arm_j5_wrist.mp3"),
            5: vlc.MediaPlayer("sounds/arm_j6_hand.mp3"),
            6: vlc.MediaPlayer("sounds/gripper.mp3"),
        }
        self.music_key_prefix = robot_name + "::music::"

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
        try:
            for i in self.players:
                play_str = self.redis_client.get(self.music_key_prefix + str(i))
                if play_str is None: continue

                play = play_str.decode("utf-8")
                if play == "play":
                    self.play(i)
                else:
                    self.stop(i)
        except redis.exceptions.ConnectionError as e:
            print(e, f" at time {time.time() : .0f}")


if __name__ == "__main__":
    player = MusicPlayer()
    while True:
        player.update()
        time.sleep(0.1)
    
