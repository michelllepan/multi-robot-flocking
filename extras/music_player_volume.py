import redis
import time
import vlc


SOUND_MAP = {
    0: "sounds/base.mp3",
    1: "sounds/arm_j0_torso.mp3",
    2: "sounds/arm_j1_shoulder.mp3",
    3: "sounds/arm_j3_elbow.mp3",
    4: "sounds/arm_j5_wrist.mp3",
    5: "sounds/arm_j6_hand.mp3",
    6: "sounds/gripper.mp3",
}
class MusicPlayer:

    def __init__(self, robot_id, redis_host, redis_port):
        super().__init__()
        robot_name = "robot_" + str(robot_id)

        self.redis_client = redis.Redis(host=redis_host, port=redis_port)
        self.music_key_prefix = robot_name + "::music::"

        self.instance = vlc.Instance()
        self.players = {}
        for i in SOUND_MAP:
            p = self.instance.media_player_new()
            m = self.instance.media_new(SOUND_MAP[i])
            p.set_media(m)
            p.audio_set_volume(100 if i == 6 else 10)
            self.players[i] = p

    def play(self, sound):
        p = self.players[sound]
        if p.get_state() != vlc.State.Playing:
            # print(p.audio_get_volume())
            p.stop()
            p.play()

    def stop(self, sound):
        p = self.players[sound]
        if p.get_state() != vlc.State.Stopped:
            p.stop()

    def update(self):
        try:
            for i in self.players:
                if i == 6:
                    state_str = self.redis_client.get("state")
                    print(state_str)
                    if state_str == "GESTURE_CLAP":
                        self.play(i)
                    else:
                        self.stop(i)
                    continue
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
    
